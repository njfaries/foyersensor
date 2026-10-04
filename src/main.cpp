/*
  Cat Detector for Foyer (PlatformIO version)
  --------------------------------------------
  Purpose: Tell the difference between a cat and a human in a hallway,
  using two TF-Luna sensors in UART mode. If only the low sensor
  triggers, and the high sensor stays clear, treat it as a cat and
  sound the buzzer. The buzzer keeps sounding until someone presses
  the silence button, or until a maximum time limit passes on its own.

  Sensor placement:
  - LOW sensor: mounted about 15-20 cm above the floor, aimed straight
    across the hallway (wall to wall), at cat-body height.
  - HIGH sensor: mounted about 150 cm above the floor, aimed straight
    across the hallway, at human-torso height.

  Wiring (per sensor, UART mode):
    TF-Luna Pin 1 (+5V)  -> ESP32 VIN / 5V
    TF-Luna Pin 2 (RXD)  -> ESP32 TX pin
    TF-Luna Pin 3 (TXD)  -> ESP32 RX pin
    TF-Luna Pin 4 (GND)  -> ESP32 GND
    TF-Luna Pin 5 (Mode) -> leave open (UART is the default mode)
    TF-Luna Pin 6        -> not connected

  Deterrent output (direct drive, low-current buzzer):
    ESP32 GPIO25 -> buzzer + terminal
    buzzer - terminal -> GND
    a snubber resistor (470 ohm to 1 kohm) across the buzzer terminals

  Silence button:
    One leg -> ESP32 GPIO (BUTTON_PIN below)
    Other leg -> GND
    No external resistor needed. The code turns on the ESP32's
    internal pull-up resistor, so the pin reads HIGH normally and
    LOW when the button is pressed.

  Debug output:
    Set DEBUG_OUTPUT to true to print live sensor values over USB
    serial (115200 baud). Set it to false for normal operation.
*/

#include <Arduino.h>
#include <HardwareSerial.h>

// A regular .cpp file, unlike a .ino file, does not get automatic
// function prototypes. These forward declarations let the functions
// below be called before their full definitions appear later in
// this file.
int readTFLuna(HardwareSerial &port);
bool updateDebounce(bool rawTriggeredNow, bool &rawTriggered,
                     unsigned long &triggerStart);
void startAlarm();
void stopAlarm();
void printDebug(unsigned long now);
const char *stateName(int s);

// ---------- User settings ----------

// Set to true to print live sensor values and trigger states.
const bool DEBUG_OUTPUT = true;

// How often a debug line is printed, in milliseconds.
const unsigned long DEBUG_INTERVAL_MS = 200;

// Serial ports for each sensor. ESP32 has Serial1 and Serial2 free
// (Serial is used for USB / debug output).
HardwareSerial LowSensor(1);   // cat-height sensor
HardwareSerial HighSensor(2);  // human-height sensor

// Pins. Change these to match your wiring.
const int LOW_RX_PIN  = 16;
const int LOW_TX_PIN  = 17;
const int HIGH_RX_PIN = 4;
const int HIGH_TX_PIN = 5;

// Output pin that drives the buzzer directly.
const int DETERRENT_PIN = 25;

// Silence button input pin. Uses the ESP32's internal pull-up,
// so the button just needs to short this pin to GND when pressed.
const int BUTTON_PIN = 27;

// Distance threshold, in cm. Set this a bit less than your hallway
// width. A reading below this value means something is breaking
// the beam. Measure your actual hallway width and adjust.
const int TRIGGER_DISTANCE_CM = 90;

// A sensor reading older than this is treated as missing, so a
// stuck value cannot keep a trigger active.
const unsigned long STALE_MS = 500;

// Debounce time for the sensors and the button. A reading must be
// steady for this long before it counts as real.
const unsigned long DEBOUNCE_MS = 100;

// Time window after the low sensor triggers. If the high sensor
// also triggers within this window, treat the event as a human.
const unsigned long HUMAN_WINDOW_MS = 1500;

// Maximum time the buzzer is allowed to sound on its own, if
// nobody presses the button. Set in milliseconds.
// Example below: 5 minutes.
const unsigned long ALARM_TIMEOUT_MS = 5UL * 60UL * 1000UL;

// Cooldown after an alarm ends, before a new event can start.
// This stops the same crossing from re-triggering immediately.
const unsigned long COOLDOWN_MS = 4000;

// ---------- State machine ----------

enum State {
  IDLE,           // waiting for the low sensor to trigger
  WAIT_FOR_HIGH,  // low sensor triggered, watching high sensor for a human
  ALARM,          // buzzer sounding, waiting for button press or timeout
  COOLDOWN        // just finished an event, waiting before watching again
};

State state = IDLE;
unsigned long stateStartTime = 0;

// Latest distance from each sensor, in cm. -1 means no frame yet.
int lowDist = -1;
int highDist = -1;

// Time of the last valid frame from each sensor.
unsigned long lowLastFrame = 0;
unsigned long highLastFrame = 0;

// Debounce tracking for each sensor
bool lowRawTriggered = false;
unsigned long lowTriggerStart = 0;
bool lowConfirmed = false;

bool highRawTriggered = false;
unsigned long highTriggerStart = 0;
bool highConfirmed = false;

// Debounce tracking for the button
bool buttonRawTriggered = false;
unsigned long buttonTriggerStart = 0;
bool buttonConfirmed = false;

// Debug print timing
unsigned long lastDebugTime = 0;

// ---------- TF-Luna frame reading ----------

// Reads one distance value from a TF-Luna UART port, in cm.
// Returns -1 if no complete, valid frame is available right now.
int readTFLuna(HardwareSerial &port) {
  // A TF-Luna UART frame is 9 bytes:
  // 0x59 0x59 Dist_L Dist_H Strength_L Strength_H Temp_L Temp_H Checksum
  static uint8_t buf[9];

  // Only proceed if a full frame's worth of bytes is waiting.
  if (port.available() < 9) {
    return -1;
  }

  // Look for the two header bytes (0x59 0x59).
  if (port.peek() != 0x59) {
    port.read();  // discard one byte and try again next loop
    return -1;
  }

  port.readBytes(buf, 9);

  if (buf[0] != 0x59 || buf[1] != 0x59) {
    return -1;  // header did not match, drop this frame
  }

  // Check the checksum: sum of first 8 bytes, low byte, must match byte 9.
  uint8_t checksum = 0;
  for (int i = 0; i < 8; i++) {
    checksum += buf[i];
  }
  if (checksum != buf[8]) {
    return -1;  // corrupt frame, drop it
  }

  int distance = buf[2] | (buf[3] << 8);
  return distance;
}

// ---------- Debounce helper ----------

// Updates a debounce state. Call this often with the latest raw
// reading. Returns true once the reading has been steady for
// DEBOUNCE_MS.
bool updateDebounce(bool rawTriggeredNow, bool &rawTriggered,
                     unsigned long &triggerStart) {
  if (rawTriggeredNow) {
    if (!rawTriggered) {
      rawTriggered = true;
      triggerStart = millis();
    }
    if (millis() - triggerStart >= DEBOUNCE_MS) {
      return true;
    }
  } else {
    rawTriggered = false;
  }
  return false;
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);  // debug output over USB

  LowSensor.begin(115200, SERIAL_8N1, LOW_RX_PIN, LOW_TX_PIN);
  HighSensor.begin(115200, SERIAL_8N1, HIGH_RX_PIN, HIGH_TX_PIN);

  pinMode(DETERRENT_PIN, OUTPUT);
  digitalWrite(DETERRENT_PIN, LOW);

  // INPUT_PULLUP turns on the ESP32's own internal pull-up resistor,
  // so no external resistor is needed for the button.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Cat detector starting up.");
  if (DEBUG_OUTPUT) {
    Serial.print("Debug output ON. Trigger distance: ");
    Serial.print(TRIGGER_DISTANCE_CM);
    Serial.println(" cm");
    Serial.println("Format: [state] LOW dist/trig  HIGH dist/trig  BTN  frame age");
    Serial.println("trig: T = confirmed, t = raw only, . = clear");
  }
}

// ---------- Main loop ----------

void loop() {
  unsigned long now = millis();

  // Read every waiting frame from each sensor. This empties the
  // buffer each pass, so the stored value is always the newest one.
  int d;
  while ((d = readTFLuna(LowSensor)) >= 0) {
    lowDist = d;
    lowLastFrame = now;
  }
  while ((d = readTFLuna(HighSensor)) >= 0) {
    highDist = d;
    highLastFrame = now;
  }

  // A reading counts only if it is fresh and below the threshold.
  bool lowFresh = (lowDist >= 0) && (now - lowLastFrame <= STALE_MS);
  bool highFresh = (highDist >= 0) && (now - highLastFrame <= STALE_MS);

  bool lowRawNow = lowFresh && lowDist > 0 && lowDist < TRIGGER_DISTANCE_CM;
  bool highRawNow = highFresh && highDist > 0 && highDist < TRIGGER_DISTANCE_CM;

  lowConfirmed = updateDebounce(lowRawNow, lowRawTriggered, lowTriggerStart);
  highConfirmed = updateDebounce(highRawNow, highRawTriggered, highTriggerStart);

  // The button reads LOW when pressed, because of the pull-up.
  bool buttonRawNow = (digitalRead(BUTTON_PIN) == LOW);
  buttonConfirmed = updateDebounce(buttonRawNow, buttonRawTriggered, buttonTriggerStart);

  switch (state) {

    case IDLE:
      if (lowConfirmed) {
        // Something broke the low beam. Start the human-check window.
        state = WAIT_FOR_HIGH;
        stateStartTime = now;
        Serial.println("Low sensor triggered. Watching for high sensor...");
      }
      break;

    case WAIT_FOR_HIGH:
      if (highConfirmed) {
        // High sensor also triggered in time. Call this a human.
        Serial.println("High sensor also triggered. Human detected. No alarm.");
        state = COOLDOWN;
        stateStartTime = now;
      } else if (now - stateStartTime >= HUMAN_WINDOW_MS) {
        // Window expired with no high trigger. Call this a cat.
        Serial.println("No high sensor trigger in time. Cat detected. Alarm!");
        startAlarm();
        state = ALARM;
        stateStartTime = now;
      }
      break;

    case ALARM:
      if (highConfirmed) {
        Serial.println("Human detected. Silencing alarm.");
        stopAlarm();
        state = COOLDOWN;
        stateStartTime = now;
      } else if (now - stateStartTime >= ALARM_TIMEOUT_MS) {
        Serial.println("Alarm timed out with no button press. Silencing alarm.");
        stopAlarm();
        state = COOLDOWN;
        stateStartTime = now;
      }
      break;

    case COOLDOWN:
      if (now - stateStartTime >= COOLDOWN_MS) {
        state = IDLE;
      }
      break;
  }

  if (DEBUG_OUTPUT && (now - lastDebugTime >= DEBUG_INTERVAL_MS)) {
    lastDebugTime = now;
    printDebug(now);
  }
}

// ---------- Debug output ----------

const char *stateName(int s) {
  switch (s) {
    case IDLE:          return "IDLE";
    case WAIT_FOR_HIGH: return "WAIT_FOR_HIGH";
    case ALARM:         return "ALARM";
    case COOLDOWN:      return "COOLDOWN";
    default:            return "?";
  }
}

// Prints one line with the state, both distances, trigger flags,
// button state, and the age of the last frame from each sensor.
void printDebug(unsigned long now) {
  char lowText[8];
  char highText[8];
  if (lowDist >= 0) snprintf(lowText, sizeof(lowText), "%d", lowDist);
  else snprintf(lowText, sizeof(lowText), "---");
  if (highDist >= 0) snprintf(highText, sizeof(highText), "%d", highDist);
  else snprintf(highText, sizeof(highText), "---");

  char lowFlag = lowConfirmed ? 'T' : (lowRawTriggered ? 't' : '.');
  char highFlag = highConfirmed ? 'T' : (highRawTriggered ? 't' : '.');

  unsigned long lowAge = (lowDist >= 0) ? (now - lowLastFrame) : 0;
  unsigned long highAge = (highDist >= 0) ? (now - highLastFrame) : 0;

  char line[128];
  snprintf(line, sizeof(line),
           "[%s] LOW %scm/%c  HIGH %scm/%c  BTN %s  age L%lums H%lums%s%s",
           stateName(state),
           lowText, lowFlag,
           highText, highFlag,
           (digitalRead(BUTTON_PIN) == LOW) ? "ON" : "off",
           lowAge, highAge,
           (lowDist >= 0 && lowAge > STALE_MS) ? "  LOW STALE" : "",
           (highDist >= 0 && highAge > STALE_MS) ? "  HIGH STALE" : "");
  Serial.println(line);
}

// ---------- Buzzer control ----------

void startAlarm() {
  digitalWrite(DETERRENT_PIN, HIGH);
}

void stopAlarm() {
  digitalWrite(DETERRENT_PIN, LOW);
}
