// Include the Wire library for I2C communication
#include <Wire.h>  

// I2C between main board(esp32) and slvae board(avr) works as follows:

/* FOR RELAYS
************************************
S aaaaaaaW cccccccc 0000dddd P
************************************
,where
S – start condition // Wire.begin()
aaaaaaa – slave address of the board // 0x58
W – write mode, should be 0 // 0x58 shifted left once to 01011000->10110000
cccccccc – command code, should be 0×10 // Wire.write(0x10) (or any other commmand code)
dddd – bitmap of the output states, i.e. bit0 corresponds to REL1, bit1 to REL2 and so on. '1' // Wire.write()
switches the relay ON, '0' switches to OFF state.
P – Stop condition // Wire.endTransmission()
*/


// I2C Slave address
#define UEXT_I2C_ADDRESS  0x58

// I2C command byte to set relays on/off
#define UEXT_COMMAND_SET_RELAYS 0x10 

/* FOR OPTI-ISOLATORS
************************************
S aaaaaaaW cccccccc P S aaaaaaaR 0000dddd P
************************************
,where
S – start condition
aaaaaaa – slave address of the board
W – write mode, should be 0
cccccccc – command code, should be 0×20
P – Stop condition
R – read mode, should be 1
dddd – bitmap of the input states received from the MOD-IO board, i.e. bit0 corresponds to IN1,
bit1 to IN2 and so on. '1' means that power is applied to the optocoupler, '0' means the opposite.
*/

// I2C Command byte for reading opto-isolator values
#define UEXT_COMMAND_READ_INPUTS 0x20 

/* FOR ANALOG IN
************************************
S aaaaaaaW cccccccc P S aaaaaaaR dddddddd 000000dd P
************************************
,where
S – start condition
aaaaaaa – slave address of the board
W – write mode, should be 0
cccccccc – command code, should be 0×30 for AIN1, 0×31 for AIN2, 0×31 for AIN3, 0×31 for
AIN4.
P – Stop condition
R – read mode, should be 1
dddddddd 000000dd – Little Endian (LSB: MSB) 10bit binary encoded value corresponding to the
input voltage. Range is 0 – 0×3FF and voltage on the pin is calculated using the following simple
formula: voltage = (3.3 / 1024) * (read value) [Volts]
// I2C Command to read analog input. Values are provided as bytes per analog in.
*/
#define UEXT_COMMAND_READ_ANALOG_AIN1 0x30 // Command code to read AIN1
#define UEXT_COMMAND_READ_ANALOG_AIN2 0x31 // Command code to read AIN2
#define UEXT_COMMAND_READ_ANALOG_AIN3 0x32 // Command code to read AIN3
#define UEXT_COMMAND_READ_ANALOG_AIN4 0x33 // Command code to read AIN4


// MAIN BOARD

// Main Board Relay Pins
const int MAIN_BOARD_RELAYS[4] = {10, 11, 22, 23}; 

// Main Board Opto Input Pins
const int MAIN_BOARD_OPTO_INPUTS[4] = {2, 3, 4, 5}; 

// Mappings to easier relay manipulation
const int MAIN_POWER_RELAY = 1;
const int MOTOR_A_RELAY = 2;
const int MOTOR_B_RELAY = 3;


const int DRIVE_BUTTON_OPTO =  10;


//UEXT BOARD

// UEXT Board Opto Input Pins
const int UEXT_OPTO_INPUTS[4] = {6, 7, 8, 9}; 

// Mapping to reasier relay manipulation
// Light relays 
const int GREEN_LIGHT_RELAY = 6;
const int RED_LIGHT_RELAY = 7; 



// Constant values for height difference in motors 
const float ERROR_DIFFERENCE = 0.16; // corresponds to 9cm difference, calculated 3.15/179cm*9cm 
const float LEVELING_DIFFERENCE = 0.035; //corresponds to 2cm difference, calculted 3.15/179cm*2cm

const int COMMANDLEN = 7;

// Function to set a relay on either the main board (0-3) or the UEXT board (4-7)
void setRelay(uint8_t relayNumber, bool state) {
    if (relayNumber < 4) {
        // Handle main board relays (relay 0-3)
        digitalWrite(MAIN_BOARD_RELAYS[relayNumber], state ? HIGH : LOW);
        Serial.print("Main board relay ");
        Serial.print(relayNumber);
        Serial.println(state ? " ON" : " OFF");
    } else if (relayNumber < 8) { 
        // Handle UEXT board relays (relay 4-7)
        static uint8_t relayState = 0; // Track current relay state on UEXT

        // Adjust relay number to 0-3 for UEXT board
        uint8_t uextRelayNumber = relayNumber - 4;

        // Set the corresponding bit in relayState based on relayNumber
        if (state) {
            relayState |= (1 << uextRelayNumber); // Set the relay bit to 1
        } else {
            relayState &= ~(1 << uextRelayNumber); // Clear the relay bit to 0
        }

        // Update relay states via I2C
        Wire.beginTransmission(UEXT_I2C_ADDRESS);
        Wire.write(UEXT_COMMAND_SET_RELAYS);
        Wire.write(relayState); // Send updated relay state to UEXT board
        Wire.endTransmission();

        Serial.print("UEXT board relay ");
        Serial.print(uextRelayNumber);
        Serial.println(state ? " ON" : " OFF");
    } 
}

// Function to read the state of all opto-isolators (0-7)
uint8_t readOptoInputs() {
    uint8_t state = 0;

    // Read main board opto inputs (0-3)
    for (uint8_t i = 0; i < 4; i++) {
        state |= (digitalRead(MAIN_BOARD_OPTO_INPUTS[i]) << i);
    }

    // Read UEXT board opto inputs (4-7)
    Wire.beginTransmission(UEXT_I2C_ADDRESS);
    Wire.write(UEXT_COMMAND_READ_INPUTS); // Send command to read opto inputs on UEXT
    Wire.endTransmission();

    // Read the input state from UEXT board
    Wire.requestFrom(UEXT_I2C_ADDRESS, 1); // Request 1 byte of data
    if (Wire.available()) {
        uint8_t uextOptoState = Wire.read(); // Get the state of UEXT opto-isolators
        state |= (uextOptoState << 4); // Shift UEXT states to the upper bits
    }

    return state; // Return the combined state of all opto-isolators (0-7)
}

// Function to read an analog input
float readUEXTAnalog(uint8_t analogInput) {
    // Ensure analogInput is between 0 and 3 (AIN1 - AIN4)
    if (analogInput > 3) {
        return -1.0; // Invalid input
    }
    
    uint8_t command = UEXT_COMMAND_READ_ANALOG_AIN1 + analogInput; // Get the correct command code

    // Start I2C communication to request the analog value
    Wire.beginTransmission(UEXT_I2C_ADDRESS);
    Wire.write(command); // Send the command to read the specified analog input
    Wire.endTransmission();
    
    delay(10); // Short delay for processing

    // Request 2 bytes of data from the UEXT board
    Wire.requestFrom(UEXT_I2C_ADDRESS, 2);
    
    uint8_t l_byte = 0, h_byte = 0;
    if (Wire.available() >= 2) {
        l_byte = Wire.read(); // Read the low byte (LSB)
        h_byte = Wire.read(); // Read the high byte (MSB)
    }

    // Convert to 10-bit ADC value (LSB:MSB)
    uint16_t adcValue = ((h_byte & 0x03) << 8) | l_byte;

    // Convert ADC value to voltage (0-3.3V range)
    float voltage = (3.3 / 1023) * adcValue;

    return voltage; // Return the calculated voltage
}


void Conf() {
  Serial.println("We are now in conf mode!");
  Serial.println("Starting calibration...");

  // Set red light on 
  setRelay(RED_LIGHT_RELAY, HIGH);

  // Set main power relay on
  setRelay(MAIN_POWER_RELAY, HIGH);

  Serial.println("Tell to turn the knob for moving down.");




}


bool waitSerialCommand(int timeToWait, char* command) {
  // This function waits for serial command, and if the wanted command is not given over serial in the given timeframe, it continues executing the program that called this function. 
  int bufferIndex = 0;
  char buffer[COMMANDLEN];

  bool hasCommand = false;

  // Store the starting time
  unsigned long startTime = millis();  

  // Loop until the time runs out
  while (millis() - startTime < timeToWait * 1000) { 
    // Calculate the remaining time
    int remainingTime = timeToWait - (millis() - startTime) / 1000;

    // Print the remaining time
    Serial.print(remainingTime);
    Serial.print("...");
    Serial.println();

    // Check if there are available bytes in the serial buffer (buffersize = 64 bytes)
    if (Serial.available()) {
      while (Serial.available()) {
        char ch = Serial.read();

        
        if (ch == '\r' || ch == '\n') {
          // Null terminate the buffer
          buffer[bufferIndex] = 0; 
          
          if (strcmp(buffer, command) == 0) {
            hasCommand = true;
            break;
          } else {
            // If the command is not what was wanted, clear buffer and continue
            Serial.println("Invalid command!");
          }

          // Reset buffer and buffer index for next command
          bufferIndex = 0;
          memset(buffer, 0, sizeof(buffer));  
        } else {
          // Add character to the buffer if not newline or carriage return
          // Ensure no buffer overflow
          if (bufferIndex < COMMANDLEN - 1) {  
            buffer[bufferIndex++] = ch;
          }
        }
      }
    }

    // If command was received, return true
    if (hasCommand) {
      return true;
    }

    // Delay for 1 second between iterations
    delay(1000);  
  }

  // Return false if time runs out without receiving the command
  return false;  
}



// Initialize
void setup() {
    // Initialize I2C (esp32-c6 as controller on the bus)   
    Wire.begin(); 

    // Set all mainboard relay pins as output, and set all of them LOW
    for (int i = 0; i < 4; i++) {
        pinMode(MAIN_BOARD_RELAYS[i], OUTPUT);
        digitalWrite(MAIN_BOARD_RELAYS[i], LOW); // Start with all relays off
    }

    // Set all UEXT board relays off

    for (int i = 4; i < 8; i++) {
      setRelay(i, LOW);
    }

    // Serial connection settings
    Serial.begin(115200); 
    // General delay so nothing is missed in serial
    delay(3000);

    Serial.println("Starting the program...");

    // Setup first time calibration. If "conf" is not send to esp32-C6-evb serial input in 15 seconds, it continues to normal operations. If "conf" is sent, it does calibration.
    Serial.println("Waiting setup command");
    Serial.println("If setup command not received in 15s, continuing to normal usage...");
    if (waitSerialCommand(15, "conf")) {
      Conf();
    }

   
    
}

// Example usage in the loop
void loop() {

}
    
   