
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>

static const int MACRO_BUTTON_COUNT = 8;
static const int COMMANDS_PER_MACRO = 16;

static const uint16_t IR_RECEIVER_PIN = 25;
static const uint16_t IR_EMITTER_PIN = 33;
static const int STATUS_LED_PIN_R = 32;
static const int STATUS_LED_PIN_G = 14;
static const int STATUS_LED_PIN_B = 13;
static const int MACRO_BUTTON_PINS[MACRO_BUTTON_COUNT] = {
  22, 21, 19, 18, 5, 4, 2, 15
};


static const uint16_t IR_RECEIVER_BUFFER_SIZE = 1024;
static const uint8_t IR_RECEIVER_TIMEOUT = 50;
static const uint8_t IR_RECEIVER_TOLERANCE = 75;  // kTolerance is normally 25%


static const int STATE_READY_TO_RECORD = 0;
static const int STATE_RECORDING = 1;
static const int STATE_READY_TO_PLAYBACK = 2;

static const int RECORD_MODE_SWITCH_INDEX = 0;
static const int RECORD_MODE_SWITCH_PIN = 23;

static const int DEBOUNCE_DELAY_MS = 50;

IRsend irsend(IR_EMITTER_PIN);
IRrecv irrecv(IR_RECEIVER_PIN, IR_RECEIVER_BUFFER_SIZE, IR_RECEIVER_TIMEOUT, true);
decode_results results;  // Somewhere to store the results

// Yucky global state, but this is all in one file anyway!
// Current state of the controller
int state = -1;
// Tracks the next time each button's state can change to prevent bouncing.
uint32_t debouncingUntil[MACRO_BUTTON_COUNT];
uint32_t recordSwitchDebouncingUntil;
bool buttonPressed[MACRO_BUTTON_COUNT];


// The button that's currently being recorded.
int recordingButtonId = -1;

struct SavedCommand {
  uint16_t size;
  uint64_t value;
  decode_type_t protocol;
};
// Stores commands that are currently being recorded
SavedCommand commands[COMMANDS_PER_MACRO];
// A pointer to the position after the final recorded command. Once this is 16, the command array is full.
short commandPtr = 0;
// A struct to store saved commands. These could be newly recorded or retrieved from flash storage.
struct {
  SavedCommand savedMacros[MACRO_BUTTON_COUNT][COMMANDS_PER_MACRO];
  short macroLength[MACRO_BUTTON_COUNT]; // 0 for unsaved commands
} saveData;

void setup() {
  Serial.begin(115200);
  Serial.println("Started!\n");

  // irrecv.setUnknownThreshold(kMinUnknownSize);
  irrecv.setTolerance(IR_RECEIVER_TOLERANCE);  // Override the default tolerance.
  irrecv.enableIRIn();  // Start the IR receiver
  irrecv.pause(); // Pause until we actually need to read

  irsend.begin();
  
  pinMode(STATUS_LED_PIN_R, OUTPUT);
  pinMode(STATUS_LED_PIN_G, OUTPUT);
  pinMode(STATUS_LED_PIN_B, OUTPUT);

  pinMode(IR_EMITTER_PIN, OUTPUT);
  pinMode(IR_RECEIVER_PIN, INPUT);

  // Enable the mode switch
  pinMode(RECORD_MODE_SWITCH_PIN, INPUT);

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

// Here be debouncing
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

// Status LED controls
void setStatus(bool r, bool g, bool b) {
  digitalWrite(STATUS_LED_PIN_R, r);
  digitalWrite(STATUS_LED_PIN_G, g);
  digitalWrite(STATUS_LED_PIN_B, b);
}

void resetStatus() {
  digitalWrite(STATUS_LED_PIN_R, false);
  digitalWrite(STATUS_LED_PIN_G, false);
  digitalWrite(STATUS_LED_PIN_B, false);
}

// Logging
int logStateNext = 0;
double maxInput = -1;
void logState() {
  double input = analogRead(IR_RECEIVER_PIN);
  if (input > maxInput) {
    maxInput = input;
  }
  
  if (millis() > logStateNext) {
    Serial.print("State is ");
    Serial.println(state);
    Serial.print("Max input was ");
    Serial.println(maxInput);
    logStateNext = millis() + 1000;
    maxInput = -1;
  }
}

int getPressedButtonIndex() {
  int pressed = -1;
  for (int i = 0; i < MACRO_BUTTON_COUNT; i++) {
    // We if multiple buttons are somehow pressed on the same cycle, 
    // all after the first are ignored until the next cycle.
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
}

void loop() {
  logState();
  int pressedButtonIndex;

  switch (state) {
    case STATE_READY_TO_RECORD:
      // Switch flipped off
      if (debounced(RECORD_MODE_SWITCH_INDEX) && !digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_PLAYBACK;
        debounce(RECORD_MODE_SWITCH_PIN);
        resetStatus();
      }

      // Button pressed
      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex != -1) {
        irrecv.resume();
        recordingButtonId = pressedButtonIndex;
        digitalWrite(STATUS_LED_PIN_R, HIGH);
        state = STATE_RECORDING;
        setStatus(true, false, false);
      }

      break;
    case STATE_RECORDING: 
      // IR command received
      if (irrecv.decode(&results) && commandPtr < 16) {
        Serial.println("I'm here!");
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

          command.size = getCorrectedRawLength(&results);
          command.value = results.command;
          command.protocol = results.decode_type;
          commands[commandPtr] = command;

          commandPtr++;

          setStatus(false, false, true);
          delay(100);
          setStatus(true, false, false);
        }
      }
    
      // Switch flipped off
      if (debounced(RECORD_MODE_SWITCH_INDEX) && !digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_PLAYBACK;
        debounce(RECORD_MODE_SWITCH_PIN);
        resetStatus();
      }

      // Recording button pressed again
      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex == recordingButtonId) {
        state = STATE_READY_TO_RECORD;
        recordingButtonId = -1;

        memcpy(saveData.savedMacros[recordingButtonId], commands, commandPtr);
        saveData.macroLength[recordingButtonId] = commandPtr;

        setStatus(true, true, false);
      }

      // These are all required if we move to any other state
      if (state != STATE_RECORDING) {
        irrecv.pause();
        commandPtr = 0;
      }
      
      break;
    case STATE_READY_TO_PLAYBACK:
      // Switch flipped on
      if (debounced(RECORD_MODE_SWITCH_INDEX) && digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_RECORD;
        debounce(RECORD_MODE_SWITCH_PIN);

        setStatus(true, true, false);
      }

      // Button pressed
      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex != -1) {
        setStatus(false, false, true);

        delay(1000);

        for (int i = 0; i < saveData.macroLength[pressedButtonIndex]; i++) {
          if (!irsend.send(
            saveData.savedMacros[pressedButtonIndex][i].protocol, 
            saveData.savedMacros[pressedButtonIndex][i].value, 
            saveData.savedMacros[pressedButtonIndex][i].size
            )
          ) {
            setStatus(true, false, false);
            delay(200);
            resetStatus();
            delay(200);
            setStatus(true, false, false);
            delay(200);
            resetStatus();
            delay(200);
            setStatus(true, false, false);
            delay(200);
            resetStatus();
            delay(200);
          } else {
            delay(500);
          }
        }

        resetStatus();
      }
      break;
  }
}
