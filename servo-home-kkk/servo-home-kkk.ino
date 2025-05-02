#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Servo Configuration ---
#define SERVO_FREQ 50 // Standard servo frequency (Hz)
#define NUM_SERVOS 3  // We are controlling 3 servos

// Speed Control (ms). Smaller=Faster
#define MOVE_DELAY_MS 6 // Default delay for user commands. Set slow for cases like 0->100 to reduce overshoot
#define HOME_DELAY_MS 4 // Different speed for 'home' command. Set faster since only 0->50 and 100->50

// --- Individual Calibration Data ---
const int calibratedPulses[NUM_SERVOS][3] = {
  {200, 326, 466}, // Servo 0: Value 0=200, 50=326, 100=466
  {185, 314, 451}, // Servo 1: Value 0=185, 50=314, 100=451
  {217, 343, 481}  // Servo 2: Value 0=217, 50=343, 100=481
};

// --- Absolute Pulse Limits (Derived from Calibration Data) ---
int servoMinLimit[NUM_SERVOS];
int servoMaxLimit[NUM_SERVOS];

// --- State Variables ---
int currentPulse[NUM_SERVOS]; // Tracks the last commanded pulse

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for serial port to connect.

  Serial.println("PCA9685 - Calibrated Servo Control (0-100) - Initializing...");
  Serial.println("---------------------------------------------------------------");

  // Calculate and store absolute min/max limits
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoMinLimit[i] = min(calibratedPulses[i][0], min(calibratedPulses[i][1], calibratedPulses[i][2]));
    servoMaxLimit[i] = max(calibratedPulses[i][0], max(calibratedPulses[i][1], calibratedPulses[i][2]));
    Serial.print("Servo "); Serial.print(i);
    Serial.print(" | Limits: ["); Serial.print(servoMinLimit[i]);
    Serial.print(", "); Serial.print(servoMaxLimit[i]);
    Serial.print("] | Cal Points: ");
    Serial.print(calibratedPulses[i][0]); Serial.print("(0) ");
    Serial.print(calibratedPulses[i][1]); Serial.print("(50) ");
    Serial.print(calibratedPulses[i][2]); Serial.println("(100)");
  }

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // --- Instant Initialization to Center Calibration Point (Value 50) ---
  Serial.println("\nSetting servos instantly to center point (Value 50)...");
  for (int channel = 0; channel < NUM_SERVOS; channel++) {
    int targetMiddlePulse = map(50, 0, 50, calibratedPulses[channel][2], calibratedPulses[channel][1]);
    Serial.print(" Setting Ch "); Serial.print(channel);
    Serial.print(" instantly to pulse: "); Serial.println(targetMiddlePulse);
    setServoPulseInstantly(channel, targetMiddlePulse);
    delay(10);
  }

  Serial.println("\nInitialization complete. Ready for commands.\n");
  Serial.println("Enter command: value1,value2,value3 (0-100) OR 'home'");
  Serial.println("  e.g. '0,50,100' or '10,,90' or 'home'");
  Serial.println("---------------------------------------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    Serial.print("Received: ["); Serial.print(input); Serial.println("]");

    bool commandProcessed = false; // Flag to track if any action was taken

    // Check for "home" command first
    if (input == "home") {
      Serial.println("Processing Command: home");
      homeServos(); // Uses slow move (BLOCKING)
      commandProcessed = true; // Mark that homing finished

    } else {
      // --- Try parsing the multi-value command ---
      int firstCommaIndex = input.indexOf(',');
      int secondCommaIndex = -1;
      int thirdCommaIndex = -1; // Check for an extra comma

      if (firstCommaIndex != -1) {
        // Find second comma *after* the first one
        secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);
        if (secondCommaIndex != -1) {
            // Check if a third comma exists *after* the second one
            thirdCommaIndex = input.indexOf(',', secondCommaIndex + 1);
        }
      }

      // Check if we found exactly two commas AND no third comma
      if (firstCommaIndex != -1 && secondCommaIndex != -1 && thirdCommaIndex == -1) {
        // --- Format is correct, proceed with parsing ---
        String valueStrings[NUM_SERVOS];
        valueStrings[0] = input.substring(0, firstCommaIndex);
        valueStrings[1] = input.substring(firstCommaIndex + 1, secondCommaIndex);
        valueStrings[2] = input.substring(secondCommaIndex + 1); // Now safe, no trailing comma

        Serial.println("Processing Multi-Value Command:");
        bool anyServoMoved = false; // Track if at least one servo was commanded

        // Process each value string for the corresponding servo
        for (int channel = 0; channel < NUM_SERVOS; channel++) {
          String currentValueStr = valueStrings[channel];
          currentValueStr.trim();

          Serial.print(" Servo "); Serial.print(channel); Serial.print(": ");

          if (currentValueStr.length() == 0) {
            Serial.println("Skipped (empty value).");
            continue;
          }

          bool conversionFailed = false;
          int value = currentValueStr.toInt();
          if (value == 0 && currentValueStr != "0") {
             conversionFailed = true;
          }

          if (conversionFailed) {
              Serial.print("ERROR - Invalid number: '");
              Serial.print(currentValueStr); Serial.println("'. Skipping.");
          } else if (value < 0 || value > 100) {
            Serial.print("ERROR - Value out of range (0-100): ");
            Serial.print(value); Serial.println(". Skipping.");
          } else {
            // Valid value, check if move is needed before calling
            int currentTargetPulse;
            int pulseAtValue0   = calibratedPulses[channel][0];
            int pulseAtValue50  = calibratedPulses[channel][1];
            int pulseAtValue100 = calibratedPulses[channel][2];
            if (value <= 50) {
                currentTargetPulse = map(value, 0, 50, pulseAtValue100, pulseAtValue50);
            } else {
                currentTargetPulse = map(value, 50, 100, pulseAtValue50, pulseAtValue0);
            }
            currentTargetPulse = constrain(currentTargetPulse, servoMinLimit[channel], servoMaxLimit[channel]);

            if (currentPulse[channel] != currentTargetPulse) {
                Serial.print("Target Value "); Serial.print(value);
                Serial.print(" -> Moving...");
                setServoValue(channel, value); // Uses slow move (BLOCKING)
                anyServoMoved = true;
                Serial.println(" -> DONE");
            } else {
                Serial.print("Target Value "); Serial.print(value);
                Serial.println(" -> Already at position.");
            }
          }
        } // end for loop through channels

        if (anyServoMoved) {
            commandProcessed = true;
        } else {
             Serial.println("No servo movement required for this command.");
        }

      } else {
        // Incorrect format (wrong number of commas)
        Serial.println("ERROR: Invalid command format. Use: value1,value2,value3 (exactly two commas) OR home");
      }
    } // end else (not "home")

    // Print READY if a command successfully completed its blocking actions
    if (commandProcessed) {
        Serial.println("======================= READY TO RECORD =======================");
    }

    Serial.println("---------------------------------------------------------------");
  } // end if Serial.available()
} // end loop()


// =========================================================================
// Helper Functions
// =========================================================================

/**
 * @brief Moves all servos to their calibrated "50" position.
 *        Uses the HOME_DELAY_MS for speed control.
 */
void homeServos() {
  Serial.print(" Homing to Value 50");
  Serial.print(" -> Homing all servos...");
  for (int channel = 0; channel < NUM_SERVOS; channel++) {
    int targetPulse = map(50, 0, 50, calibratedPulses[channel][2], calibratedPulses[channel][1]);
    moveServoPulseSlowly(channel, targetPulse, HOME_DELAY_MS);
  }
  Serial.println(" -> Homing DONE");
}

/**
 * @brief Maps a value (0-100) to a target pulse using INVERSE piecewise linear
 *        interpolation and initiates a slow move to that pulse.
 */
void setServoValue(uint8_t channel, int value) {
  if (channel >= NUM_SERVOS) return;
  int clampedValue = constrain(value, 0, 100);
  int pulseAtValue0   = calibratedPulses[channel][0];
  int pulseAtValue50  = calibratedPulses[channel][1];
  int pulseAtValue100 = calibratedPulses[channel][2];
  int targetPulse;
  if (clampedValue <= 50) {
    targetPulse = map(clampedValue, 0, 50, pulseAtValue100, pulseAtValue50);
  } else {
    targetPulse = map(clampedValue, 50, 100, pulseAtValue50, pulseAtValue0);
  }
  moveServoPulseSlowly(channel, targetPulse, MOVE_DELAY_MS);
}

/**
 * @brief Moves a servo from its current pulse to a target pulse incrementally. BLOCKING.
 */
void moveServoPulseSlowly(uint8_t channel, int targetPulse, int stepDelayMillis) {
  if (channel >= NUM_SERVOS) return;
  int clampedTargetPulse = constrain(targetPulse, servoMinLimit[channel], servoMaxLimit[channel]);
  int startPulse = currentPulse[channel];
  if (clampedTargetPulse == startPulse) return;
  int step = (clampedTargetPulse > startPulse) ? 1 : -1;
  for (int pulse = startPulse; pulse != clampedTargetPulse; pulse += step) {
      setServoPulseInstantly(channel, pulse + step);
      delay(stepDelayMillis);
  }
}

/**
 * @brief Sets the raw pulse width for a servo instantly. Clamps pulse and updates state.
 */
void setServoPulseInstantly(uint8_t channel, int pulse) {
  if (channel >= NUM_SERVOS) return;
  int clampedPulse = constrain(pulse, servoMinLimit[channel], servoMaxLimit[channel]);
  pwm.setPWM(channel, 0, clampedPulse);
  currentPulse[channel] = clampedPulse;
}
