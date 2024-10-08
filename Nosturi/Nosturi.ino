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
const int LIMIT_DOWN_A = 0;
const int LIMIT_UP_A = 1;
const int LIMIT_DOWN_B = 2;
const int LIMIT_UP_B = 3;

// Leveling errors
const float ERROR_DIFFERENCE = 0.16;      // corresponds to 9cm difference, calculated 3.15/179cm*9cm
const float LEVELING_DIFFERENCE = 0.035;  //corresponds to 2cm difference, calculted 3.15/179cm*2cm

// Button states
volatile bool EMERGENCY_BUTTON_STATE = false;
volatile bool DRIVE_BUTTON_STATE = false;
volatile int DIRECTION = 0b00; // 01 up, 10 down, 00/11 shouldn't be possible (wiring error)

// Debounce button inputs
unsigned long debouncingTime = 30;
volatile unsigned long lastMicros;

// Function to read the state of all opto-isolators (0-7)
uint8_t readOptoInputs() {
  uint8_t state = 0;

  // Read main board opto inputs (0-3)
  for (uint8_t i = 0; i < 4; i++) {
    state |= (digitalRead(MAIN_BOARD_OPTO_INPUTS[i]) << i);
  }

  // Read UEXT board opto inputs (4-7)
  Wire.beginTransmission(UEXT_I2C_ADDRESS);
  Wire.write(UEXT_COMMAND_READ_INPUTS);  // Send command to read opto inputs on UEXT
  Wire.endTransmission();

  // Read the input state from UEXT board
  Wire.requestFrom(UEXT_I2C_ADDRESS, 1);  // Request 1 byte of data
  if (Wire.available()) {
    uint8_t uextOptoState = Wire.read();  // Get the state of UEXT opto-isolators
    state |= (uextOptoState << 4);        // Shift UEXT states to the upper bits
  }

  return state;  // Return the combined state of all opto-isolators (0-7)
}
// Function to read an analog input
float readUEXTAnalog(uint8_t analogInput) {
  // Ensure analogInput is between 0 and 3 (AIN1 - AIN4)
  if (analogInput > 3) {
    return -1.0;  // Invalid input
  }

  uint8_t command = UEXT_COMMAND_READ_ANALOG_AIN1 + analogInput;  // Get the correct command code

  // Start I2C communication to request the analog value
  Wire.beginTransmission(UEXT_I2C_ADDRESS);
  Wire.write(command);  // Send the command to read the specified analog input
  Wire.endTransmission();

  delay(10);  // Short delay for processing

  // Request 2 bytes of data from the UEXT board
  Wire.requestFrom(UEXT_I2C_ADDRESS, 2);

  uint8_t l_byte = 0, h_byte = 0;
  if (Wire.available() >= 2) {
    l_byte = Wire.read();  // Read the low byte (LSB)
    h_byte = Wire.read();  // Read the high byte (MSB)
  }

  // Convert to 10-bit ADC value (LSB:MSB)
  uint16_t adcValue = ((h_byte & 0x03) << 8) | l_byte;

  // Convert ADC value to voltage (0-3.3V range)
  float voltage = (3.3 / 1023) * adcValue;

  return voltage;  // Return the calculated voltage
}

void setRelay(uint8_t relayNumber, bool state) {
  if (relayNumber < 4) {
    // Handle main board relays (relay 0-3)
    digitalWrite(MAIN_BOARD_RELAYS[relayNumber], state ? HIGH : LOW);
    Serial.print("Main board relay ");
    Serial.print(relayNumber);
    Serial.println(state ? " ON" : " OFF");
  } else if (relayNumber < 8) {
    // Handle UEXT board relays (relay 4-7)
    static uint8_t relayState = 0;  // T rack current relay state on UEXT

    // Adjust relay number to 0-3 for UEXT board
    uint8_t uextRelayNumber = relayNumber - 4;

    // Set the corresponding bit in relayState based on relayNumber
    if (state) {
      relayState |= (1 << uextRelayNumber);  // Set the relay bit to 1
    } else {
      relayState &= ~(1 << uextRelayNumber);  // Clear the relay bit to 0
    }

    // Update relay states via I2C
    Wire.beginTransmission(UEXT_I2C_ADDRESS);
    Wire.write(UEXT_COMMAND_SET_RELAYS);
    Wire.write(relayState);  // Send updated relay state to UEXT board
    Wire.endTransmission();

    Serial.print("UEXT board relay ");
    Serial.print(uextRelayNumber);
    Serial.println(state ? " ON" : " OFF");
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


void move() {
  // Add 250 millis delay for not overloading the motor controller
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

  // I2C init
  Wire.begin();

  // Serial init
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting the program...");


  attachInterrupt(MAIN_BOARD_OPTO_INPUTS[EMERGENCY_STOP_BUTTON_OPTO], EMERGENCY, CHANGE);
  attachInterrupt(MAIN_BOARD_OPTO_INPUTS[DRIVE_BUTTON_OPTO], DRIVE, CHANGE);

  for (int i = 0; i < 4; i++) {
    pinMode(MAIN_BOARD_RELAYS[i], OUTPUT);
    pinMode(MAIN_BOARD_OPTO_INPUTS[i], INPUT_PULLUP);
    setRelay(i, LOW);
  }

  for (int j = 4; j < 8; j++) {
    setRelay(j, LOW);
  }

  // Opto isolators states
  int currentStateOptoInputs = readOptoInputs();

  // Check if EM stop is on when powerd on
  if (currentStateOptoInputs & (1 << EMERGENCY_STOP_BUTTON_OPTO)) {
    EMERGENCY_BUTTON_STATE = true;
  }

  // Check and store direction bits
  int dirUp = (currentStateOptoInputs & 0b00000010) >> 2;
  int dirDown = (currentStateOptoInputs & 0b00000100) >> 3;

  DIRECTION = (dirUp << 1) | dirDown;

  // General delay
  delay(2000);
}


void loop() {
  if (!EMERGENCY_BUTTON_STATE) {
    setRelay(RED_LIGHT_RELAY, LOW);
    if (!DRIVE_BUTTON_STATE) {
      setRelay(GREEN_LIGHT_RELAY, HIGH);
      Serial.println("Moving...");
      Serial.println(DIRECTION, BIN);
      move();
    } else {
      setRelay(GREEN_LIGHT_RELAY, LOW);
      Serial.println("Stopping...");
      stop();
    }
  } else {
    setRelay(RED_LIGHT_RELAY, HIGH);
    Serial.println("Emergency button is pressed");
  }
}
