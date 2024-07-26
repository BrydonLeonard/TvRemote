
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>

static const int MACRO_BUTTON_COUNT = 8;
static const int COMMANDS_PER_MACRO = 16;

static const uint16_t IR_RECEIVER_PIN = 15;
static const uint16_t IR_EMITTER_PIN = 2;
static const int STATUS_LED_PIN_R = 36;
static const int STATUS_LED_PIN_G = 39;
static const int STATUS_LED_PIN_B = 34;
static const int MACRO_BUTTON_PINS[MACRO_BUTTON_COUNT] = {
  13, 12, 14, 22, 26, 25, 33, 32
};


static const uint16_t IR_RECEIVER_BUFFER_SIZE = 1024;
static const uint8_t IR_RECEIVER_TIMEOUT = 50;
static const uint8_t IR_RECEIVER_TOLERANCE = 25;  // kTolerance is normally 25%


static const int STATE_READY_TO_RECORD = 0;
static const int STATE_RECORDING = 1;
static const int STATE_READY_TO_PLAYBACK = 2;

static const int RECORD_MODE_SWITCH_INDEX = 0;
static const int RECORD_MODE_SWITCH_PIN = 4;

static const int DEBOUNCE_DELAY_MS = 50;


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

// Stores commands that are currently being recorded
uint32_t commands[COMMANDS_PER_MACRO];
// A pointer to the position after the final recorded command. Once this is 16, the command array is full.
short commandPtr = 0;
// A struct to store saved commands. These could be newly recorded or retrieved from flash storage.
struct {
  uint32_t savedMacros[MACRO_BUTTON_COUNT][COMMANDS_PER_MACRO];
  short macroLength[MACRO_BUTTON_COUNT]; // 0 for unsaved commands
} saveData;

struct SavedCommand {
  uint16_t size,
  uint64_t value,
  decode_type_t protocol,
}

void setup() {
  Serial.begin(115200);
  Serial.println("Started!\n");

  // irrecv.setUnknownThreshold(kMinUnknownSize);
  irrecv.setTolerance(IR_RECEIVER_TOLERANCE);  // Override the default tolerance.
  irrecv.enableIRIn();  // Start the receiver

  if (digitalRead(RECORD_MODE_SWITCH_PIN)) {
    state = STATE_READY_TO_RECORD;
  } else {
    state = STATE_READY_TO_PLAYBACK;
  }

  // Enable the mode switch
  pinMode(RECORD_MODE_SWITCH_PIN, INPUT);

  // Enable all buttons
  for (int i = 0; i < MACRO_BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT);
    Serial.print("Enabling reads on pin ");
    Serial.println(buttonPins[i]);
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
void statusRed(bool r, bool g, bool b) {
  digitalWrite(STATUS_LEN_PIN_R, r);
  digitalWrite(STATUS_LEN_PIN_G, g);
  digitalWrite(STATUS_LEN_PIN_B, b);
}

void statusReset() {
  digitalWrite(STATUS_LEN_PIN_R, false);
  digitalWrite(STATUS_LEN_PIN_G, false);
  digitalWrite(STATUS_LEN_PIN_B, false);
}

int logStateNext = 0;

void logState() {
  if (millis() > logStateNext) {
    Serial.print("State is ");
    Serial.println(state);
    logStateNext = millis() + 1000;
  }
}

int getPressedButtonIndex() {
  int pressed = -1;
  for (int i = 0; i < MACRO_BUTTON_COUNT; i++) {
    // We if multiple buttons are somehow pressed on the same cycle, 
    // all after the first are ignored until the next cycle.
    if (pressed == -1 && debounced(i) && !buttonPressed[i] && digitalRead(buttonPins[i])) {
      debounce(i);
      buttonPressed[i] = true;
      pressed = i;
    } else if (debounced(i) && !digitalRead(buttonPins[i])) {
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
      if (debounced(RECORD_MODE_SWITCH_INDEX) && !digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_PLAYBACK;
        debounce(RECORD_MODE_SWITCH_PIN);
      }

      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex != -1) {
        recordingButtonId = pressedButtonIndex;
        digitalWrite(STATUS_LED_PIN_R, HIGH);
        state = STATE_RECORDING;
      }

      break;
    case STATE_RECORDING: 
      // Check for decoded results _first_ so we don't risk missing too many.
      if (irrecv.decode(&results) && commandPtr < 16) {
        Serial.println(resultToSourceCode(&results));
        Serial.print("Protocol is ");
        Serial.println(results.decode_type);
        Serial.print("Raw data is ");
        uint16_t *rawArray = resultToRawArray(&results);
        Serial.println(*rawArray);
        int size = getCorrectedRawLength(&results);
        Serial.print("Result length is actually ");
        Serial.println(size);
        commands[commandPtr] = results.command;
        commandPtr++;
        delete [] rawArray;
      }
    
      // The user stopped recording
      if (debounced(RECORD_MODE_SWITCH_INDEX)) {
        if (!digitalRead(RECORD_MODE_SWITCH_PIN)) {
          state = STATE_READY_TO_PLAYBACK;
          debounce(RECORD_MODE_SWITCH_PIN);
          commandPtr = 0;

          digitalWrite(STATUS_LED_PIN_R, LOW);
        }
      }

      pressedButtonIndex = getPressedButtonIndex();
      // Finish this recording
      if (pressedButtonIndex == recordingButtonId) {
        state = STATE_READY_TO_RECORD;
        recordingButtonId = -1;

        memcpy(saveData.savedMacros[recordingButtonId], commands, commandPtr);
        saveData.macroLength[recordingButtonId] = commandPtr;
        
        commandPtr = 0;

        digitalWrite(STATUS_LED_PIN_R, LOW);
      }
      
      break;
    case STATE_READY_TO_PLAYBACK:
      if (debounced(RECORD_MODE_SWITCH_INDEX) && digitalRead(RECORD_MODE_SWITCH_PIN)) {
        state = STATE_READY_TO_RECORD;
        debounce(RECORD_MODE_SWITCH_PIN);
      }

      pressedButtonIndex = getPressedButtonIndex();
      if (pressedButtonIndex != -1) {
        for (int i = 0; i < saveData.macroLength[pressedButtonIndex]; i++) {
          // send...
        }
      }
      break;
  }
}
