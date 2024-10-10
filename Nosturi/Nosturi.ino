// Include the Wire library for I2C communication
#include <Wire.h>
// Include the Preferences for storing values in flash (non-volatile memory) for calibration and stuff
#include <Preferences.h>
Preferences preferences;

// I2C Slave address
#define UEXT_I2C_ADDRESS 0x58

// I2C command byte to set relays on/off
#define UEXT_COMMAND_SET_RELAYS 0x10
// I2C Command byte for reading opto-isolator values
#define UEXT_COMMAND_READ_INPUTS 0x20

#define UEXT_COMMAND_READ_ANALOG_AIN1 0x30
#define UEXT_COMMAND_READ_ANALOG_AIN2 0x31
#define UEXT_COMMAND_READ_ANALOG_AIN3 0x32
#define UEXT_COMMAND_READ_ANALOG_AIN4 0x33


// MAIN BOARD
const int MAIN_BOARD_RELAYS[4] = { 10, 11, 22, 23 };
const int MAIN_BOARD_OPTO_INPUTS[4] = { 1, 2, 3, 15 };
const int MAIN_POWER_RELAY = 1;
const int MOTOR_A_RELAY = 2;
const int MOTOR_B_RELAY = 3;
const int DRIVE_BUTTON_OPTO = 0;
const int EMERGENCY_STOP_BUTTON_OPTO = 1;
const int DIR_UP_OPTO = 2;
const int DIR_DOWN_OPTO = 3;


//UEXT BOARD
const int GREEN_LIGHT_RELAY = 6;
const int RED_LIGHT_RELAY = 7;

// Leveling error values
const float ERROR_DIFFERENCE = 0.16;      // corresponds to 9cm difference, 3.15/179cm*9cm
const float LEVELING_DIFFERENCE = 0.035;  //corresponds to 2cm difference, 3.15/179cm*2cm

// External hardware states
volatile bool EMERGENCY_BUTTON_STATE = false;
volatile bool DRIVE_BUTTON_STATE = false;

// Variables for direction and optoisolator state
volatile bool DIRECTION;
volatile int OPTOSTATES;


// For the first power up, we want to go to limit switches to check what pot readings are. This stores if the "calibration" has been done,  so it does not do it every time machine is powered.
volatile float MOTOR_A_UP_LIMIT;
volatile float MOTOR_A_DOWN_LIMIT;
volatile float MOTOR_B_UP_LIMIT;
volatile float MOTOR_B_DOWN_LIMIT;

// Motor potentiometer encoding pins 0 = AN1, 1 = AN2 on UEXT board
const int MOTOR_A_ANALOG = 0;
const int MOTOR_B_ANALOG = 1;

// Debounce button inputs
unsigned long debouncingTime = 30;
volatile unsigned long lastMicros;


// Function to read the state of all opto-isolators (0-7)
uint8_t readOptoInputs() {
  uint8_t state = 0;

  // Read main board opto inputs (0-3)
  for (uint8_t i = 0; i < 4; i++) {
    // Shift the input and store it in the state variable
    state |= (digitalRead(MAIN_BOARD_OPTO_INPUTS[i]) << i);
  }

  // Begin transmission to UEXT board via I2C
  Wire.beginTransmission(UEXT_I2C_ADDRESS);
  // Send command to read opto inputs on UEXT
  Wire.write(UEXT_COMMAND_READ_INPUTS);
  // End transmission
  Wire.endTransmission();

  // Request 1 byte of data from UEXT board
  Wire.requestFrom(UEXT_I2C_ADDRESS, 1);

  // Check if data is available
  if (Wire.available()) {
    // Get the state of UEXT opto-isolators
    uint8_t uextOptoState = Wire.read();
    // Shift UEXT states to the upper bits and store in the state variable
    state |= (uextOptoState << 4);
  }

  // Return the combined state of all opto-isolators (0-7)
  return state;
}

// Function to read an analog input
float readUEXTAnalog(uint8_t analogInput) {
  // Ensure analogInput is between 0 and 3 (AIN1 - AIN4)
  if (analogInput > 3) {
    // Return an invalid value if input is out of range
    return -1.0;
  }

  // Get the correct command code based on the analog input
  uint8_t command = UEXT_COMMAND_READ_ANALOG_AIN1 + analogInput;

  // Begin I2C communication to request the analog value
  Wire.beginTransmission(UEXT_I2C_ADDRESS);
  // Send the command to read the specified analog input
  Wire.write(command);
  // End transmission
  Wire.endTransmission();

  // Short delay for processing
  delay(10);

  // Request 2 bytes of data from the UEXT board
  Wire.requestFrom(UEXT_I2C_ADDRESS, 2);

  uint8_t l_byte = 0, h_byte = 0;
  // Check if at least 2 bytes are available to read
  if (Wire.available() >= 2) {
    // Read the low byte (LSB)
    l_byte = Wire.read();
    // Read the high byte (MSB)
    h_byte = Wire.read();
  }

  // Convert to 10-bit ADC value (LSB:MSB)
  uint16_t adcValue = ((h_byte & 0x03) << 8) | l_byte;

  // Convert ADC value to voltage (0-3.3V range)
  float voltage = (3.3 / 1023) * adcValue;

  // Return the calculated voltage
  return voltage;
}

// Function to set relay state
void setRelay(uint8_t relayNumber, bool state) {
  // Check if the relay is on the main board (relay 0-3)
  if (relayNumber < 4) {
    // Set or clear the relay on the main board
    digitalWrite(MAIN_BOARD_RELAYS[relayNumber], state ? HIGH : LOW);
  }
  // Check if the relay is on the UEXT board (relay 4-7)
  else if (relayNumber < 8) {
    // Track current relay state on UEXT
    static uint8_t relayState = 0;

    // Adjust relay number to 0-3 for UEXT board
    uint8_t uextRelayNumber = relayNumber - 4;

    // Set the corresponding bit in relayState based on relayNumber
    if (state) {
      // Set the relay bit to 1
      relayState |= (1 << uextRelayNumber);
    } else {
      // Clear the relay bit to 0
      relayState &= ~(1 << uextRelayNumber);
    }

    // Begin I2C transmission to update relay states
    Wire.beginTransmission(UEXT_I2C_ADDRESS);
    // Send command to set relays on UEXT
    Wire.write(UEXT_COMMAND_SET_RELAYS);
    // Send updated relay state to UEXT board
    Wire.write(relayState);
    // End transmission
    Wire.endTransmission();
  }
}




void IRAM_ATTR EMERGENCY() {
  if ((long)(micros() - lastMicros) >= debouncingTime * 1000) {
    EMERGENCY_BUTTON_STATE = !EMERGENCY_BUTTON_STATE;
    lastMicros = micros();
  }
}

void IRAM_ATTR DRIVE() {
  if ((long)(micros() - lastMicros) >= debouncingTime * 1000) {
    DRIVE_BUTTON_STATE = !DRIVE_BUTTON_STATE;
    lastMicros = micros();
  }
}

void IRAM_ATTR DIRECTION_CHANGE() {
  if ((long)(micros() - lastMicros) >= debouncingTime * 1000) {
    DIRECTION = !DIRECTION;
    lastMicros = micros();
  }
}

void level() {
  // Stops the higher position motor until the height difference is lower than 2cm
  float heightA = readUEXTAnalog(0);
  float heightB = readUEXTAnalog(1);
  // if A > B and going UP
  if ((heightA > heightB) && DIRECTION) {
    Serial.println("Leveling B ...");
    Serial.println(heightA);
    Serial.println(heightB);
    while (abs(heightA - heightB) > LEVELING_DIFFERENCE) {
      Serial.print("diff: ");
      Serial.println(abs(heightA - heightB));
      Serial.println("Stopping A, Moving B");
      setRelay(MOTOR_A_RELAY, LOW);
      heightA = readUEXTAnalog(0);
      heightB = readUEXTAnalog(1);
    }
    // If A < B and going UP
  } else if ((heightA < heightB) && DIRECTION) {
    Serial.println("Leveling A...");
    Serial.println(heightA);
    Serial.println(heightB);
    while (abs(heightA - heightB) > LEVELING_DIFFERENCE) {
      Serial.print("diff: ");
      Serial.println(abs(heightA - heightB));
      Serial.println("Stopping B, Moving A");
      setRelay(MOTOR_B_RELAY, LOW);
      heightA = readUEXTAnalog(0);
      heightB = readUEXTAnalog(1);
    }
    // A > B and going DOWN
  } else if ((heightA > heightB) && !DIRECTION) {
    Serial.println("Leveling A...");
    Serial.println(heightA);
    Serial.println(heightB);
    while (abs(heightA - heightB) > LEVELING_DIFFERENCE) {
      Serial.print("diff: ");
      Serial.println(abs(heightA - heightB));
      Serial.println("Stopping B, Moving A");
      setRelay(MOTOR_B_RELAY, LOW);
      heightA = readUEXTAnalog(0);
      heightB = readUEXTAnalog(1);
    }

    // A < B and going DOWN
  } else {
    Serial.println("Leveling B...");
    Serial.println(heightA);
    Serial.println(heightB);
    while (abs(heightA - heightB) > LEVELING_DIFFERENCE) {
      Serial.print("diff: ");
      Serial.println(abs(heightA - heightB));
      Serial.println("Stopping A, Moving B");
      setRelay(MOTOR_A_RELAY, LOW);
      heightA = readUEXTAnalog(0);
      heightB = readUEXTAnalog(1);
    }
  }
}


void move() {
  // Add 250 millis delay for not overloading the motor controller

  // Check for differences in motor positions (height)
  float heightA = readUEXTAnalog(0);
  float heightB = readUEXTAnalog(1);

  float diff = abs(heightA - heightB);
  Serial.print("diff: ");
  Serial.println(diff);

  // If the difference is 2cm per side, then stop and adjust the one side
  if (diff > LEVELING_DIFFERENCE) {
    // This level function is not going to stop both motors. It shuts down one of them for short moment, so the other one can close the gap.
    level();
  }

  // If there is more than 9 cm difference, halt the system.
  if (diff > ERROR_DIFFERENCE) {
    EMERGENCY_BUTTON_STATE = true;
  }
  setRelay(MOTOR_A_RELAY, HIGH);
  delay(250);
  setRelay(MOTOR_B_RELAY, HIGH);
}

void stop() {
  setRelay(MOTOR_A_RELAY, LOW);
  delay(250);
  setRelay(MOTOR_B_RELAY, LOW);
}


void setup() {
  // Namespace config, false = Read-Write
  preferences.begin("config", false);

  // For the first time, we want to go to limit switches. This stores if the "calibration" has been done, so it does not do it every time.
  MOTOR_A_UP_LIMIT = preferences.getFloat("MOTOR_A_UP_LIM", -1);
  MOTOR_A_DOWN_LIMIT = preferences.getFloat("MOTOR_A_DOWN_L", -1);
  MOTOR_B_UP_LIMIT = preferences.getFloat("MOTOR_B_UP_LIM", -1);
  MOTOR_B_DOWN_LIMIT = preferences.getFloat("MOTOR_B_DOWN_L", -1);

  // I2C init
  Wire.begin();

  // Serial init
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting the program...");

  // EM button
  attachInterrupt(MAIN_BOARD_OPTO_INPUTS[EMERGENCY_STOP_BUTTON_OPTO], EMERGENCY, CHANGE);

  // Drive button
  attachInterrupt(MAIN_BOARD_OPTO_INPUTS[DRIVE_BUTTON_OPTO], DRIVE, CHANGE);

  // Direction change button
  attachInterrupt(MAIN_BOARD_OPTO_INPUTS[DIR_UP_OPTO], DIRECTION_CHANGE, CHANGE);

  for (int i = 0; i < 4; i++) {
    pinMode(MAIN_BOARD_RELAYS[i], OUTPUT);
    pinMode(MAIN_BOARD_OPTO_INPUTS[i], INPUT_PULLUP);
    setRelay(i, LOW);
  }

  for (int j = 4; j < 8; j++) {
    setRelay(j, LOW);
  }

  // Opto isolators states
  OPTOSTATES = readOptoInputs();


  // Check if EM stop is on when powerd on
  if (OPTOSTATES & (1 << EMERGENCY_STOP_BUTTON_OPTO)) {
    EMERGENCY_BUTTON_STATE = true;
  }

  // Check and store direction bits at startup
  if (((OPTOSTATES >> 2) & 0x03) - 0b01 == 0b0) {
    DIRECTION = false;
  } else {
    DIRECTION = true;
  }

  // Main power on
  setRelay(MAIN_POWER_RELAY, HIGH);


  // General delay
  delay(2000);
}


void loop() {
  // Opto input states
  OPTOSTATES = readOptoInputs();
  Serial.print("Motor A Up limit voltage: ");
  Serial.println(MOTOR_A_UP_LIMIT);
  Serial.print("Motor A Down limit voltage: ");
  Serial.println(MOTOR_A_DOWN_LIMIT);
  Serial.print("Motor B Up limit voltage: ");
  Serial.println(MOTOR_B_UP_LIMIT);
  Serial.print("Motor B Down limit voltage: ");
  Serial.println(MOTOR_B_DOWN_LIMIT);

  // MOTOR_A_DOWN_LIMIT, MOTOR_B_UP_LIMIT, MOTOR_B_DOWN_LIMIT);

  // Mask with 0xF0 = 0b11110000 to only account upper 4 bits
  unsigned char upperBits = OPTOSTATES & 0xF0;


  // Emergency button state
  if (!EMERGENCY_BUTTON_STATE) {

    // If all limit switches are high its all good
    // LImit all high so motors not in limits 1111
    // When 1000/1100, LOWER switches are activated, now moving only up allowed
    // When 0001/0011, UPPER switches are activated, now moving only down allowed
    if (upperBits == 0xF0) {
      setRelay(RED_LIGHT_RELAY, LOW);

      // If drive button pushed. Motors can only move when button is pushed.
      if (!DRIVE_BUTTON_STATE) {
        setRelay(GREEN_LIGHT_RELAY, LOW);
        setRelay(RED_LIGHT_RELAY, HIGH);
        Serial.println(DIRECTION ? "Moving UP..." : "Moving DOWN...");

        // Setting motor relays on
        move();

      } else {

        // Setting motor relays off
        stop();
        setRelay(GREEN_LIGHT_RELAY, HIGH);
        setRelay(RED_LIGHT_RELAY, LOW);
        Serial.println(DIRECTION ? "Stopping... Direction is  set to UP..." : "Stopping... Direction is set to DOWN...");
      }
      // Check if EITHER of the lower switches is activated
    } else if (upperBits == 0xE0 || upperBits == 0xD0 || upperBits == 0xC0) {
      Serial.println("Lower limit switch triggered...");

      // For leveling error calculation, read analog inputs when lower limit switches activated
      if (MOTOR_A_DOWN_LIMIT < 0 || MOTOR_B_DOWN_LIMIT < 0) {
        MOTOR_A_DOWN_LIMIT = readUEXTAnalog(0);
        MOTOR_B_DOWN_LIMIT = readUEXTAnalog(1);
        preferences.putFloat("MOTOR_A_DOWN_L", MOTOR_A_DOWN_LIMIT);
        preferences.putFloat("MOTOR_B_DOWN_L", MOTOR_B_DOWN_LIMIT);
      }
      setRelay(RED_LIGHT_RELAY, HIGH);

      // Check if direction is UP, only then motors can move
      if (DIRECTION) {
        setRelay(RED_LIGHT_RELAY, LOW);
        // If drive button pushed. Motors can only move when button is pushed.
        if (!DRIVE_BUTTON_STATE) {
          setRelay(GREEN_LIGHT_RELAY, LOW);
          setRelay(RED_LIGHT_RELAY, HIGH);
          Serial.println(DIRECTION ? "Moving UP..." : "Moving DOWN...");

          // Setting motor relays on
          move();

        } else {

          // Setting motor relays off
          stop();
          setRelay(GREEN_LIGHT_RELAY, HIGH);
          Serial.println(DIRECTION ? "Stopping... Direction is set to UP..." : "Stopping... Direction is set to DOWN...");
        }
        // If direction was not up, not able to move.
      } else {
        stop();
        Serial.println("Lower limit switches activated, cant move down...");
      }
    } else if (upperBits == 0xB0 || upperBits == 0x70 || upperBits == 0x30) {
      Serial.println("Upper limit switch triggered...");

      // For leveling error calculation, read analog inputs when lower limit switches activated
      if (MOTOR_A_UP_LIMIT < 0 || MOTOR_B_UP_LIMIT < 0) {
        MOTOR_A_UP_LIMIT = readUEXTAnalog(0);
        MOTOR_B_UP_LIMIT = readUEXTAnalog(1);
        preferences.putFloat("MOTOR_A_UP_LIM", MOTOR_A_UP_LIMIT);
        preferences.putFloat("MOTOR_B_UP_LIM", MOTOR_B_UP_LIMIT);
      }

      setRelay(RED_LIGHT_RELAY, HIGH);


      // Check if direction is DOWN, only then motors can move
      if (!DIRECTION) {
        setRelay(RED_LIGHT_RELAY, LOW);
        // If drive button pushed. Motors can only move when button is pushed.
        if (!DRIVE_BUTTON_STATE) {
          setRelay(GREEN_LIGHT_RELAY, LOW);
          setRelay(RED_LIGHT_RELAY, HIGH);
          Serial.println(DIRECTION ? "Moving UP..." : "Moving DOWN...");

          // Setting motor relays on
          move();

        } else {

          // Setting motor relays off
          stop();
          setRelay(GREEN_LIGHT_RELAY, HIGH);
          Serial.println(DIRECTION ? "Stopping... Direction is set to UP..." : "Stopping... Direction is set to DOWN...");
        }
        // If direction was not up, not able to move.
      } else {
        stop();
        Serial.println("Upper limit switches activated, cant move up...");
      }
    }
  } else {

    // main power off

    setRelay(MAIN_POWER_RELAY, LOW);
    // If emergency button is pushed, stop all, and indicate that by flashing red.
    stop();
    setRelay(GREEN_LIGHT_RELAY, LOW);
    setRelay(RED_LIGHT_RELAY, HIGH);
    delay(500);
    setRelay(RED_LIGHT_RELAY, LOW);
    delay(500);
    Serial.println("Emergency button is pressed");
  }
}
