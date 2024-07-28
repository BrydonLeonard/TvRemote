
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>

#include <Preferences.h>

// <Configuration>
static const int MACRO_BUTTON_COUNT = 8; // Number of buttons on the box
static const int COMMANDS_PER_MACRO = 16; // The max commands for each macro

static const uint16_t IR_RECEIVER_PIN = 25; 
static const uint16_t IR_EMITTER_PIN = 33;
static const int STATUS_LED_PIN_R = 32;
static const int STATUS_LED_PIN_G = 14;
static const int STATUS_LED_PIN_B = 13;
// The number of pins must match MACRO_BUTTON_COUNT
static const int MACRO_BUTTON_PINS[MACRO_BUTTON_COUNT] = {
  22, 21, 19, 18, 5, 4, 2, 15
};

// Knobs for the IR receiver
static const uint16_t IR_RECEIVER_BUFFER_SIZE = 1024; // Buffer for incoming commands
static const uint8_t IR_RECEIVER_TIMEOUT = 15;  // The minimum pause between two commands
static const uint8_t IR_MIN_UNKNOWN_SIZE = 12;
static const uint8_t IR_RECEIVER_TOLERANCE = 25;  // The percentage tolerance for bad signals. 25% is the default.
// After we record a command, we don't accept new commands for a while. Doing so avoids recording a command multiple
// times when the person recording a macro holds the button for too long.
static const int IR_RECEIVER_INTER_COMMAND_PAUSE = 500; 
static const int IR_SEND_REPEATS = 1; // The number of repeats sent with IR commands

// I know states. I have the best states.
static const int STATE_READY_TO_RECORD = 0;
static const int STATE_RECORDING = 1;
static const int STATE_READY_TO_PLAYBACK = 2;

static const int RECORD_MODE_SWITCH_PIN = 23;

// https://en.wikipedia.org/wiki/Switch#Contact_bounce
static const int DEBOUNCE_DELAY_MS = 50;

// Only check the switch this often during recording
static const int SWITCH_SCAN_INTERVAL = 1000;

// The length of the breaks between commands during playback
static const int COMMAND_PLAYBACK_INTERVAL = 750;

// </Configuration>

// <Globals>
Preferences preferences; // For saving macros

IRsend irsend(IR_EMITTER_PIN);
IRrecv irrecv(IR_RECEIVER_PIN, IR_RECEIVER_BUFFER_SIZE, IR_RECEIVER_TIMEOUT, true);
decode_results results;  // Somewhere to store the commands received by the IR receiver.
// Used alonside IR_RECEIVER_INTER_COMMAND_PAUSE to record when commands should be accepted again.
uint32_t receiver_paused_until = 0; 

int state = -1;
// Tracks the next time each button's state can change to prevent bouncing.
uint32_t debouncingUntil[MACRO_BUTTON_COUNT];
uint32_t recordSwitchDebouncingUntil;
bool buttonPressed[MACRO_BUTTON_COUNT];

// Used to avoid checking the recording switch on every loop
uint32_t nextSwitchScan;

// The button that's currently being recorded.
int recordingButtonId = -1;

struct SavedCommand {
  uint8_t size; // 1b
  uint64_t value; // 8b
  decode_type_t protocol; // 1b
};

// Stores commands that are currently being recorded
SavedCommand commands[COMMANDS_PER_MACRO];
// Used while recording - points to the index _after_ the final command in the currently recording macro
// (i.e. this stores the length of the current macro).
short commandPtr = 0;
// A struct to store saved commands. These could be newly recorded or retrieved from flash storage.
struct {
  SavedCommand savedMacros[MACRO_BUTTON_COUNT][COMMANDS_PER_MACRO];
  short macroLength[MACRO_BUTTON_COUNT];  // 0 for unsaved commands
} saveData;

// </Globals>

/*
 * Loads macros from flash memory.
 * 
 * See stashNewMacro for more information on how macros are saved.
 */
void loadMacros() {
  for (int i = 0; i < MACRO_BUTTON_COUNT; i++) {
    String keyPrefix = String(i, HEX); 
    String lengthKey = keyPrefix + "l";

    char lengthKeyCharArr[lengthKey.length() + 1];
    lengthKey.toCharArray(lengthKeyCharArr, lengthKey.length() + 1);

    uint8_t macroLength = preferences.getUChar(lengthKeyCharArr, 0);

    saveData.macroLength[i] = macroLength;

#ifdef DEBUG 
    if (macroLength > 0) {
      Serial.print("Found a saved macro for button ");
      Serial.println(i);
    } else {
      Serial.print("Found no saved macro in ");
      Serial.println(lengthKey);
    }
#endif

    for (int j = 0; j < macroLength; j++) {
      String commandKey = keyPrefix + String(j, HEX);

      char keyCharArr[commandKey.length() + 1];
      commandKey.toCharArray(keyCharArr, commandKey.length() + 1);

      uint64_t encodedCommand = preferences.getULong64(keyCharArr, 0);

      SavedCommand decodedCommand;
      decodeCommand(encodedCommand, &decodedCommand);

      saveData.savedMacros[i][j].value = decodedCommand.value;
      saveData.savedMacros[i][j].protocol = decodedCommand.protocol;
      saveData.savedMacros[i][j].size = decodedCommand.size;
    }
  }
}

/*
 * Saves the button's macro to flash memory
 * 
 * This uses the Preferences library (https://tinyurl.com/esp32-preferences) to save the macro
 * so it can be retrieved after a power cycle. The preferences library stores key-value pairs,
 * so this method define a set of standard keys per macro. The keys are kept to two characters
 * to minimize their storage overhead:
 * - "{buttonIndex}l" - Stores the length of the macro (in commands)
 * - "{buttonIndex}{commandIndex}" - Stores the encoded command at index {commandIndex} for the 
 *   macro on button {buttonIndex}.
 */
void stashNewMacro(int macroButtonIndex) {
  String keyPrefix = String(macroButtonIndex, HEX); 
  String lengthKey = keyPrefix + "l";

  char lengthKeyCharArr[lengthKey.length() + 1];
  lengthKey.toCharArray(lengthKeyCharArr, lengthKey.length() + 1);

  preferences.putUChar(lengthKeyCharArr, saveData.macroLength[macroButtonIndex]);

#ifdef DEBUG
  Serial.print("Saving macro length in ");
  Serial.println(lengthKeyCharArr);
#endif

  for (int i = 0; i < saveData.macroLength[macroButtonIndex]; i++) {
    String commandKey = keyPrefix + String(i, HEX);

    char keyCharArr[commandKey.length() + 1];
    commandKey.toCharArray(keyCharArr, commandKey.length() + 1);

    uint64_t encodedCommand = encodeCommand(saveData.savedMacros[macroButtonIndex][i]);
    preferences.putULong64(keyCharArr, encodedCommand);

#ifdef DEBUG
    Serial.print("Saving macro command in ");
    Serial.println(keyCharArr);
#endif
  }
}

/*
 * Nice pretty printing of commands.
 */
void print(SavedCommand command) {
#ifdef DEBUG
  Serial.print("Size: ");
  Serial.print(command.size);
  Serial.print(", Protocol: ");
  Serial.print(command.protocol);
  Serial.print(", Value: ");
  Serial.print(command.value);
  Serial.print(", Hex: ");
  Serial.println(String(command.value, HEX));
#endif
}

/*
 * Encodes a command as a uint64.
 * 
 * The most significant 16 bits are used to store the command's size and protocol (8 bits each).
 * All remaining bits then store the command value (the actual command that's transmitted to the
 * device). This approach assumes that no command will actually be more than 32 bits; my testing
 * has shown that to be the case, but if the code encounters some protocol that uses extra long 
 * commands, this'll fail horribly. I decided that it's worth the tradeoff since doing it this
 * way gives us extra space in the ESP32's flash memory to store commands.
 */
uint64_t encodeCommand(SavedCommand command) {
  print(command);
  uint64_t size = command.size;
  size = size << 56;
  uint64_t protocol = command.protocol;
  protocol = protocol << 48;
  uint64_t value = command.value;

  uint64_t encoded = value | protocol | size;

  return encoded;
}

/*
 * Decodes 
 * 
 * This is just the opposite of encodeCommand. See that function's description for more info.
 */
void decodeCommand(uint64_t encodedCommand, SavedCommand* command) {
  uint64_t mask = 0b1111111111111111;
  mask = mask << 48;
  mask = ~mask;

  command->value = encodedCommand & mask;
  command->size = encodedCommand >> 56;
  command->protocol = static_cast<decode_type_t>((encodedCommand >> 48) & 0b0000000011111111);
}

void setup() {
  Serial.begin(115200);

#ifdef DEBUG
  while (!Serial)
    delay(50);
  Serial.println("Started!\n");
#endif

  preferences.begin("tv-remote", false);
  loadMacros();

  pinMode(IR_EMITTER_PIN, OUTPUT);
  pinMode(IR_RECEIVER_PIN, INPUT);

  // Tweak a couple knobs on the IR receiver and start it
  irrecv.setUnknownThreshold(IR_MIN_UNKNOWN_SIZE);
  irrecv.setTolerance(IR_RECEIVER_TOLERANCE);
  irrecv.enableIRIn();

  // Start the IR emitter
  irsend.begin();

  // These are the status LED's three pins
  pinMode(STATUS_LED_PIN_R, OUTPUT);
  pinMode(STATUS_LED_PIN_G, OUTPUT);
  pinMode(STATUS_LED_PIN_B, OUTPUT);

  // Enable the mode switch
  pinMode(RECORD_MODE_SWITCH_PIN, INPUT_PULLDOWN);

  // This makes sure we start in the right state
  if (digitalRead(RECORD_MODE_SWITCH_PIN)) {
    state = STATE_READY_TO_RECORD;
    setStatus(true, true, false);
  } else {
    state = STATE_READY_TO_PLAYBACK;
  }

  // Enable all buttons
  for (int i = 0; i < MACRO_BUTTON_COUNT; i++) {
    pinMode(MACRO_BUTTON_PINS[i], INPUT);
    Serial.print("Enabling reads on pin ");
    Serial.println(MACRO_BUTTON_PINS[i]);
  }
}

// <BUTTONS AND DEBOUNCING>
bool debounced(int buttonId) {
  return millis() > debouncingUntil[buttonId];
}

void debounce(int buttonId) {
  debouncingUntil[buttonId] = millis() + DEBOUNCE_DELAY_MS;
}

bool recordSwitchDebounced() {
  return millis() > recordSwitchDebouncingUntil;
}

void debounceRecordSwitch() {
  recordSwitchDebouncingUntil = millis() + DEBOUNCE_DELAY_MS;
}

int getPressedButtonIndex() {
  // We really can't afford to spend time scanning unnecessarily because it messes with the IR
  // receiver. As such, when recording, this only checks the button for which we're actively
  // recording. Otherwise we scan all of them.
  if (recordingButtonId == -1) {
    int pressed = -1;
    for (int i = 0; i < MACRO_BUTTON_COUNT; i++) {
      // If multiple buttons are somehow pressed on the same cycle, we ignore all except the first one
      // we encounter here.
      if (pressed == -1 && debounced(i) && !buttonPressed[i] && digitalRead(MACRO_BUTTON_PINS[i])) {
        debounce(i);
        buttonPressed[i] = true;
        pressed = i;
      } else if (debounced(i) && !digitalRead(MACRO_BUTTON_PINS[i])) {
        debounce(i);
        buttonPressed[i] = false;
      }
    }
    return pressed;
  } else {
    if (debounced(recordingButtonId) && !buttonPressed[recordingButtonId] && digitalRead(MACRO_BUTTON_PINS[recordingButtonId])) {
      debounce(recordingButtonId);
      buttonPressed[recordingButtonId] = true;
      return recordingButtonId;
    } else if (debounced(recordingButtonId) && !digitalRead(MACRO_BUTTON_PINS[recordingButtonId])) {
      debounce(recordingButtonId);
      buttonPressed[recordingButtonId] = false;
    }
  }

  return -1;
}
// </BUTTONS AND DEBOUNCING>

// <STATUS LED CONTROLS>
void setStatus(bool r, bool g, bool b) {
  analogWrite(STATUS_LED_PIN_R, 10 * r);
  analogWrite(STATUS_LED_PIN_G, 10 * g);
  analogWrite(STATUS_LED_PIN_B, 10 * b);
}

void resetStatus() {
  analogWrite(STATUS_LED_PIN_R, false);
  analogWrite(STATUS_LED_PIN_G, false);
  analogWrite(STATUS_LED_PIN_B, false);
}
// </STATUS LED CONTROLS>

void loop() {
  int pressedButtonIndex;

  switch (state) {
    case STATE_READY_TO_RECORD:
      // Switch flipped off
      if (recordSwitchDebounced() && !digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_PLAYBACK;
        debounceRecordSwitch();
        resetStatus();
      }

      // Button pressed
      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex != -1) {
#ifdef DEBUG
        Serial.print("Pressed button ");
        Serial.println(pressedButtonIndex);
#endif
        irrecv.resume();
        recordingButtonId = pressedButtonIndex;
        state = STATE_RECORDING;
        setStatus(true, false, false);
      }

      break;
    case STATE_RECORDING:
      // IR command received
      if (irrecv.decode(&results) && millis() > receiver_paused_until && commandPtr < 16) {
        // -1 is UNKNOWN. We can't accept any of those because we need the protocol
        // in order to transmit the command later. Flash the status LED red to show
        // that something's gone wrong.
        if (results.decode_type == -1) {
          setStatus(false, false, false);
          delay(100);
          setStatus(true, false, false);
          delay(100);
          setStatus(false, false, false);
          delay(100);
          setStatus(true, false, false);
        } else {
          SavedCommand command;

          command.size = results.bits;
          command.value = results.value;
          command.protocol = results.decode_type;
          commands[commandPtr] = command;
          
#ifdef DEBUG
          Serial.println(resultToHumanReadableBasic(&results));
          Serial.println("Encoded command is:");
          print(command);
#endif

          commandPtr++;

          // Flash blue!
          setStatus(false, false, true);
          delay(100);
          setStatus(true, false, false);
        }

        // We won't accept any more commands until this time
        receiver_paused_until = millis() + IR_RECEIVER_INTER_COMMAND_PAUSE;
      }
      yield;

      // Recording button pressed again
      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex == recordingButtonId) {
        state = STATE_READY_TO_RECORD;

        for (int i = 0; i < commandPtr; i++) {
          SavedCommand command = commands[i];
          saveData.savedMacros[recordingButtonId][i].size = command.size;
          saveData.savedMacros[recordingButtonId][i].value = command.value;
          saveData.savedMacros[recordingButtonId][i].protocol = command.protocol;
        }
        saveData.macroLength[recordingButtonId] = commandPtr;

        stashNewMacro(recordingButtonId);

        recordingButtonId = -1;

        setStatus(true, true, false);
      }

      yield;

      if (millis() > nextSwitchScan) {
        // Switch flipped off
        if (recordSwitchDebounced() && !digitalRead(RECORD_MODE_SWITCH_PIN)) {
          state = STATE_READY_TO_PLAYBACK;
          debounceRecordSwitch();
          resetStatus();
        }

        nextSwitchScan = millis() + SWITCH_SCAN_INTERVAL;
      }

      // These are all required if we move to any other state
      if (state != STATE_RECORDING) {
        irrecv.pause();
        commandPtr = 0; // This effectively clears out the stored macro
      }

      break;
    case STATE_READY_TO_PLAYBACK:
      // Switch flipped on
      if (recordSwitchDebounced() && digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_RECORD;
        debounceRecordSwitch();

        setStatus(true, true, false);
        return;
      }

      // Button pressed
      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex != -1) {
        setStatus(false, false, true);

        for (int i = 0; i < saveData.macroLength[pressedButtonIndex]; i++) {
          irsend.send(
                saveData.savedMacros[pressedButtonIndex][i].protocol,
                saveData.savedMacros[pressedButtonIndex][i].value,
                saveData.savedMacros[pressedButtonIndex][i].size,
                IR_SEND_REPEATS);
          
          if (i < saveData.macroLength[pressedButtonIndex] - 1) {
            // Delay between commands in case the receiving device needs some time
            delay(COMMAND_PLAYBACK_INTERVAL);
          }
        }

        resetStatus();
      }
      break;
  }
}
