// WearDuino Core Demo
/* Embedded Firmware for the WearDuino, hardware version 005
Copyright (c) 2015, Mark Leavitt DBA Wearable Health Labs LLC. All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution:
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
This version supports the LSM9DS0 9DOF sensor, currently only for accelerometer and gyro (not magnetometer)
Pins and sensors are configured by sending an array (deviceConfig) to the WearDuino. See "Wearduino Communication Protocol" document.
*/


#include <RFduinoBLE.h>
#include <Wire.h>
// Global constants for hardwired pins and configurable pin count
const byte pinRedLED = 0;
const byte pinGreenLED = 1;
const byte configPinCount = 5;
// Global constants for memory chip size and location of config data 
const long memSize = 0x0001FFFF;
const long deviceConfigAddress = 0x0001FF00;
// Default sensor config array: Mode=Streaming, Timing=150 mSec, accel=onWhenConn, gyro=off, mag=off, redLED=OFF, greenLED=ON when connected, pins(2,3,4)=OFF 
byte factorydeviceConfig[] = { 0x01, 0x0F, 0x02, 0x00, 0x00, 0x03, 0x07, 0x03, 0x03, 0x03, 0, 0, 0, 0, 0, 0, 0, 0}; 
byte deviceConfig[18];
// Pin configuration arrays
bool isOutputPin[configPinCount];
bool isDigitalPin[configPinCount];
bool isOnWhenConnPin[configPinCount];
bool isOnDuringMeasPin[configPinCount];
int analogOutValuePin[configPinCount];
// Accel, gyro, and mag configuration
bool isActivatedAccel;
bool isActivatedGyro;
bool isActivatedMag;
// Global state and timing variables
bool isStreaming = false;
bool isLogging = false;
bool isStandby = true;
bool isConnected = false;
uint64_t cycleInterval = 150;								// cycleInterval is the time between readings, in milliseconds

void setup() {
  Wire.begin();
  pinMode(pinGreenLED,OUTPUT);
	digitalWrite(pinGreenLED, LOW);
	pinMode(pinRedLED,OUTPUT);
	digitalWrite(pinRedLED, LOW);
	eepromRead(deviceConfigAddress, 18, deviceConfig);		// Recall deviceConfig from EEPROM

	configureDevice(deviceConfig);							// Configure sensors based on default deviceConfig values
	RFduinoBLE.deviceName = "WearDuino";					// Before starting radio, set up device name, etc.
	RFduinoBLE.customUUID = "30dff168-62de-11e4-b116-123b93f75cba";			// UUID for the Core Demo (currently called Streaming in the Demo App)
	RFduinoBLE.advertisementData = "";						// No advertisement data (other than device name)
	RFduinoBLE.advertisementInterval = 300;					// 300 mSec between advertisements; should be able to find device within 1 second
	RFduinoBLE.txPowerLevel = +4;							// Options range from -20dbM to +4 dBm
	RFduinoBLE.begin();										// Start the radio stack, which will begin advertising
}
void loop(){
  if (!isConnected || isStandby) {							// if Bluetooth connection is not active, or if mode is Standby, just go to sleep for the cycle interval                           
    RFduino_ULPDelay(MILLISECONDS(cycleInterval));
    return;
  }
  else{														// otherwise, Bluetooth connection is active, so...  
    uint64_t startTime = millis();							// remember this moment in time  
		controlPins('M');									// Control any output pins into the Measurement state
		byte buffer[6];										// prepare a data buffer to hold accel or gyro results
		char btString[20];									// prepare the bluetooth transmission string buffer of 20 bytes
		btString[0] = 0x00;									// First byte is 0x00 for a streaming measurement
		byte memString[32];									// prepare the memory string for logging
		memString[0] = 0;									// TODO store TIME in initial bytes of memory
		
		if (isActivatedAccel) {								// If accelerometer is active...
			readAccel(buffer);								// read the accelerometer data and place in buffer...
			for (int i = 0; i < 6; i++){					// move the data to slots 1...6 the bluetooth transmission string
				btString[i + 1] = (char)buffer[i];
			}
		}	else {
			for (int i = 0; i < 6; i++){					// Otherwise, fill spaces with 0x00
				btString[i + 1] = 0x00;
			}
		}
		if (isActivatedGyro) {								// If gyro is active...
			readGyro(buffer);								// read the gyro data and place in buffer...
			for (int i = 0; i < 6; i++){					// and move the data to the bluetooth transmission string and update position pointer
				btString[i + 7] = (char)buffer[i];
			}
		}	else {
			for (int i = 0; i < 6; i++){					// Otherwise, fill spaces with 0x00
				btString[i + 7] = 0x00;
			}
		}
		byte pinInputs = 0x00;								// Prepare a byte to hold the result of pin inputs
		if (isActivatedMag) {								// If magnetometer is active...
			pinInputs = 0x80;								// Set high byte of pinInputs ON (so we'll know Mag is active)
			readMag(buffer);								// read the mag data and place in buffer...
			for (int i = 0; i < 6; i++){					// and move the data to the bluetooth transmission string and update position pointer
				btString[i + 13] = (char)buffer[i];
			}
		}	else {
			for (int i = 0; i < 6; i++){					// Otherwise, fill spaces with 0x00
				btString[i + 13] = 0x00;
			}
		}

		if (!isActivatedMag) {										// If magnetometer is NOT active, check for analog inputs 
			for (int pinNum = 2; pinNum < 5; pinNum++){				// Only check pins 2, 3 and 4
				if (!isOutputPin[pinNum] && !isDigitalPin[pinNum]){	// If this is an analog input, then
					int analogValue = analogRead(pinNum);			// Read the analog input
					int lowBytePosition = 9 + 2 * pinNum;			// Calculate position in btString; Pin 2 starts at Byte 13
					btString[lowBytePosition] = lowByte(analogValue);
					btString[lowBytePosition + 1] = highByte(analogValue);
				}
			}
		}	
		
		for (int pinNum = 0; pinNum < 5; pinNum++){					// Now check all pins for digital input configuration
			if (!isOutputPin[pinNum] && isDigitalPin[pinNum]){		// If it is a digital input,
				byte pinValue = digitalRead(pinNum);				// Read it into pinValue
				pinInputs = pinInputs | ((pinValue & 0x01) << pinNum);	// Shift left into the appropriate position, then OR it with pinInputs
			}
		}
		btString[19] = pinInputs;							// pinInputs now holds digital input values, put it in btString last position
		
		if (isStreaming) {									// If streaming is active...
			RFduinoBLE.send(btString, 20);					// ...transmit as a BLE notification
		}
		if (isLogging) {									// If logging is active...
			// TODO logging to memory
		}
		controlPins('C');									// Control any output pins back into the Connected state
		uint64_t duration = millis() - startTime;			// Determine how long it took to do the measurement
    if (duration >= cycleInterval) duration = 0;			// (Handle rare case of 50-day rollover or hung measurement (duration > cycleInterval))
    RFduino_ULPDelay(MILLISECONDS(cycleInterval - duration));    // Subtract measurement time from cycle time to get remaining delay in ULP mode
		return;
  }
}

// Bluetooth radio event routines
void RFduinoBLE_onAdvertisement(bool start){
}
void RFduinoBLE_onConnect(){
  isConnected = true;
	controlPins('C');										// Control output pins into the connected state
	controlAccel('C');										// Control accel into the connected state
	controlGyro('C');										// Control gyro into the connected state
	controlMag('C');
}
void RFduinoBLE_onDisconnect(){
  isConnected = false;
	controlPins('D');										// Control any output pins into the disconnected state
	controlAccel('D');										// Control accel into the disconnected state
	controlGyro('D');										// Control gyro into the disconnected state
	controlMag('D');
}
void RFduinoBLE_onRSSI(int rssi){							// returns the received signal strength after connection (-0dBm to -127dBm)
}
void RFduinoBLE_onReceive(char * data, int len)	{			// Called when command string is received by the radio
	// Command byte = 0x00: Device configuration command
	if (data[0] == 0x00) {
		for (int i = 1; i < len; i++) {
			deviceConfig[i - 1] = (byte)data[i];			// Copy payload (beginning with byte 1) to deviceConfig array (could use memcpy?)
		}
		eepromWrite(deviceConfigAddress, 18, deviceConfig);	// storing deviceConfig in EEPROM location
		configureDevice(deviceConfig);						// perform the configuration
	}
	// Command byte = 0x04: Report configuration
	if (data[0] == 0x04){
		char btString[20];
		btString[0] = 0x04;									// Echo back command type 
		for (int i = 0; i < 18; i++){
			btString[i + 1] = deviceConfig[i];				// Copy device config array into the transmission string after the message byte
		}
		RFduinoBLE_send(btString, 20);						// And transmit it
	}
	// TODO handle sync and other commands

	// Command byte = 0xFF: reset to factory configuration and reboot
	if (data[0] == 0xFF) {
		memcpy(deviceConfig, factorydeviceConfig, 18);		// Recall the factory configuration 
		eepromWrite(deviceConfigAddress, 18, deviceConfig);	// storing deviceConfig in EEPROM location
		RFduino_ULPDelay(250);								// Wait to be sure EEPROM storage is complete
		RFduino_systemReset();								// And reboot
	}
	// Command byte = 0x06...0x07: write and read EEPROM (testing purposes only)
	if (data[0] == 0x06){									// Command byte 0x06: Write to memory
		long memAddress;
		memAddress = data[1] + (data[2] << 8);
		byte dataToMemorize[16];
		for (int i = 0; i < 16; i++){
			dataToMemorize[i] = data[i + 3];
		}
		eepromWrite(memAddress, 16, dataToMemorize);
	}
	if (data[0] == 0x07){									// Command byte 0x07: Read from memory
		long memAddress = 0;
		char btString[20];
		btString[0] = 0x07;									// Echo back command type
		memAddress = memAddress + data[1] + (data[2] << 8);
		byte dataRecalled[16];
		eepromRead(memAddress, 16,dataRecalled);
		for (int i = 0; i < 16; i++){
			btString[i+1] = dataRecalled[i];
		}
		RFduinoBLE_send(btString, 18);						// transmit it
	}

}

// Configuration of device
		/* Cheat Sheet for GPIO pin configuration (bytes 5 through 10 in deviceConfig)
		00 = analog input
		02 = digital input
		03 = digital output, always off
		X5 = analog output (X = high byte), on when connected
		07 = digital output, on when connected
		X9 = analog output (X = high byte), on during measurement
		0B = digital output, on only during measurement
		XD = analog output, on when connected and during measurement
		0F = digital output, on when connected and during measurement */
void configureDevice(byte deviceConfig[]){

	// Configure mode
	switch (deviceConfig[0]) {														// Switch to the appropriate mode, based on byte 0 of config array
	case 0x00:																		// Command 0x00 - turn off streaming and logging																		
		isStreaming = false;
		isLogging = false;
		isStandby = true;
		break;
	case 0x01:																		// Command 0x01: Streaming only
		isStreaming = true;
		isLogging = false;
		isStandby = false;
		break;
	case 0x02:																		// Command 0x02: Logging only
		isStreaming = false;
		isLogging = true;
		isStandby = false;
		break;
	case 0x03:																		// Command 0x03: both Streaming and Logging ON
		isStreaming = true;
		isLogging = true;
		isStandby = false;
		break;
	}
	// Timing configuration
	if (deviceConfig[1] > 0) {
		cycleInterval = 10 * deviceConfig[1];										// set cycleInterval in mSec to be 10 times value of timing config byte
	}
	else{
		cycleInterval = 10;															// don't allow a cycleInterval less than 10 mSec
	}
	// Accelerometer configuration
	if (deviceConfig[2] > 0) {														// Set isActivatedAccel if config byte is > 0 
		isActivatedAccel = 1;
	}
	else {
		isActivatedAccel = 0;
	}
	// Gyroscope configuration
	if (deviceConfig[3] > 0) {														// Set isActivatedGyro if config byte is > 0 
		isActivatedGyro = 1;
	}
	else {
		isActivatedGyro = 0;
	}										
	// Magnetometer configuration
	if (deviceConfig[4] > 0) {														// Set isActivatedMag if config byte is > 0 
		isActivatedMag = 1;
	}
	else { 
		isActivatedMag = 0;
	}											
	// GPIO pin configurations
	for (int pinNum = 0; pinNum < configPinCount; pinNum++) {
		int deviceConfigIndex = pinNum + 5;											// Pin config starts at byte 6
		// TODO -- add error checking to reject invalid pin settings?
		isOutputPin[pinNum] = deviceConfig[deviceConfigIndex] & 0x01;				// if bit 0 is on, it is an output (otherwise input)
		isDigitalPin[pinNum] = deviceConfig[deviceConfigIndex] & 0x02;				// if bit 1 is on, it is digital (otherwise analog)
		isOnWhenConnPin[pinNum] = deviceConfig[deviceConfigIndex] & 0x04;			// if bit 2 is on, it is on when connected (applies only for digital output)
		isOnDuringMeasPin[pinNum] = deviceConfig[deviceConfigIndex] & 0x08;			// if bit 3 is on, it is on during measurements (applies only for digital output)
		analogOutValuePin[pinNum] = (deviceConfig[deviceConfigIndex] & 0xF0) << 4;	// the high 4 bits are multiplied by 16 to get the analog output value (0 - 240)
		if (isOutputPin[pinNum]) {													// Set the pin mode for this pin
			pinMode(pinNum, OUTPUT);
		}
		else {
			pinMode(pinNum, INPUT);
		}
	}
}

// GPIO Pin Control function
void controlPins(char state){
	for (int pinNum = 0; pinNum < configPinCount; pinNum++) {						// Go through this for each GPIO pin...
		if (isOutputPin[pinNum]) { 													// Only needed for output pins
			switch (state) {														// Perform appropriate actions for Measurement, Connected, or Disconnected states
			case 'M':																// Measurement state
				if (isDigitalPin[pinNum]){											// If it's digital...
					digitalWrite(pinNum, isOnDuringMeasPin[pinNum]);				// turn it on only if it's supposed to be on during measurements
				}
				else if (isOnDuringMeasPin[pinNum]) {								// Otherwise, it's analog. Is it supposed to be on during Measurement...			
					analogWrite(pinNum, analogOutValuePin[pinNum]);					// then set it to the analog out value.  
				}
				else {
					analogWrite(pinNum, 0);											// otherwise set analog output to zero.
				}
				break;
			case 'C':																// Connected state
				if (isDigitalPin[pinNum]){											// If it's digital...
					digitalWrite(pinNum, isOnWhenConnPin[pinNum]);					// turn it on only if it's supposed to be on during Connected state
				}
				else if (isOnWhenConnPin[pinNum]) {									// Otherwise, it's analog. Is it supposed to be on when Connected...			
					analogWrite(pinNum, analogOutValuePin[pinNum]);					// then set it to the analog out value.  
				}
				else {
					analogWrite(pinNum, 0);											// otherwise set analog output to zero.
				}
				break;
			case 'D':																// Disconnected state
				if (isDigitalPin[pinNum]){											// If it's digital...
					digitalWrite(pinNum, LOW);										// turn it off
				}
				else {
					analogWrite(pinNum, 0);											// otherwise, it's analog. Set analog output to zero.
				}
			}																		// End of switch statement block
		}																			// End of if(isOutput) block
	}																				// End of for/next loop	
}

// Accelerometer functions
void controlAccel(char state) {
	const byte ACCELADDRESS = 0x1D;													// For LSM9DSO with SDO wired high, 7-bit i2C address=0x1D
	const byte POWER_CTL_REG = 0x20;												// Power control is via CTRL_REG1_XM (0x20)
	const byte POWER_CTL_VAL = 0x37;												// Power control value 0x37 turns on 12.5 hz operation, all axes
	switch (state) {																// Perform appropriate actions for Connected or Disconnected states
		case 'C':																	// Connected state
			writeRegister(ACCELADDRESS, POWER_CTL_REG, POWER_CTL_VAL);
			break;
		case 'D':																	// Disconnected state
			writeRegister(ACCELADDRESS, POWER_CTL_REG, 0x00);
	}																				// End of switch statement block
}
void readAccel(byte _buffer[]) {
	const byte ACCELADDRESS = 0x1D;													// For LSM9DSO with SDO wired high, 7-bit i2C address=0x1D
	const byte DATAX0 = 0xA8;														// X-Axis Data 0 register is 0x28; must set MSB on, so it's 0xA8
	readRegister(ACCELADDRESS, DATAX0, 0x06, _buffer);								// Read 6 bytes of data, returning them in buffer 
}

// Gyro functions
void controlGyro(char state) {
	const byte GYROADDRESS = 0x6B;													// For LSM9DSO with SDO high, 7-bit i2C address=0x6B
	const byte GYRO_POWER_CTL_REG = 0x20;											// Power control is via CTRL_REG1_G (0x20)
	const byte GYRO_POWER_CTL_VAL = 0x0F;											// Will power on gyro at 95 Hz, 12.5 Hz filter cutoff
	switch(state) {																	// Power gyro on or off as requested
		case 'C':
			writeRegister(GYROADDRESS, GYRO_POWER_CTL_REG, GYRO_POWER_CTL_VAL);
			break;
		case 'D':
			writeRegister(GYROADDRESS, GYRO_POWER_CTL_REG, 0x00);
	}																				// End of switch statement block
}
void readGyro(byte _buffer[]) {
	const byte GYROADDRESS = 0x6B;													// For LSM9DSO with SDO wired high, 7-bit i2C address=0x6B
	const byte GYRO_DATAX0 = 0xA8;													// X-Axis Data 0 register is 0x28; must set MSB on so it's 0xA8
	readRegister(GYROADDRESS, GYRO_DATAX0, 0x06, _buffer);							// Read 6 bytes of data, returning them in buffer 
}

// Magnetometer functions
void controlMag(char state) {
	const byte MAGADDRESS = 0x1D;													// For LSM9DSO with SDO high, 7-bit i2C address=0x1D
	const byte MAG_POWER_CTL_REG = 0x26;											// Power control is via CTRL_REG7_XM (0x26)
	const byte MAG_POWER_CTL_VAL = 0x00;											// Will power on continuous conversion mode
	const byte MAG_FULLSCALE_CTL_REG = 0x25;										// Full-scale control is via CTRL_REG6_XM (0x25)
	const byte MAG_FULLSCALE_CTL_VAL = 0x00;										// Sets full scale to +-2 Gauss
	const byte MAG_RATE_CTL_REG = 0x24;												// Rate control is via CTRL_REG5_XM (0x24)
	const byte MAG_RATE_CTL_VAL = 0x08;												// Rate 12.5 Hz 
	switch(state) {																	// Power MAG on or off as requested
		case 'C':
			writeRegister(MAGADDRESS, MAG_FULLSCALE_CTL_REG, MAG_FULLSCALE_CTL_VAL);
			writeRegister(MAGADDRESS, MAG_RATE_CTL_REG, MAG_RATE_CTL_VAL);
			writeRegister(MAGADDRESS, MAG_POWER_CTL_REG, MAG_POWER_CTL_VAL);
			break;
		case 'D':
			writeRegister(MAGADDRESS, MAG_POWER_CTL_REG, 0x02);
	}																				// End of switch statement block
}
void readMag(byte _buffer[]) {
	const byte MAGADDRESS = 0x1D;													// For LSM6DSO with SDO wired high, 7-bit i2C address=0x6B
	const byte MAG_DATAX0 = 0x88;													// X-Axis Data 0 register is 0x08; must set MSB on so it's 0x88
	readRegister(MAGADDRESS, MAG_DATAX0, 0x06, _buffer);							// Read 6 bytes of data, returning them in buffer 
}


// i2C register access functions
void writeRegister(int device, byte registerAddress, byte value){
	Wire.beginTransmission(device);
	Wire.write(registerAddress);
	Wire.write(value);
	Wire.endTransmission();
}
void readRegister(int device, byte registerAddress, int numBytes, byte buffer[]){
	// Function readRegister will communicate with i2C device, reading numBytes starting from a registerAddress and storing them in array buffer[]
	Wire.beginTransmission(device);
	Wire.write(registerAddress);						// First set the register start address 
	Wire.endTransmission();
	Wire.beginTransmission(device);
	Wire.requestFrom(device, numBytes);					// New read the data bytes	
	int i = 0;
	while (i < numBytes)	{
		buffer[i] = Wire.read();
		i++;
	}
	Wire.endTransmission();
}
void testRegister(int device){
	//Function testRegister simply tests whether a specific i2c address is "live"
	Wire.beginTransmission(device);
	Wire.endTransmission();
}

// EEPROM access functions
void eepromWrite(long memAddress, byte length, byte _bytesToWrite[]) {
	int eepromDeviceAddress = 0x50;						// 7-bit i2c address for Atmel AT24CM01; 8-bit version is 0xA0
	byte memAccessBuffer[34];							// Prepare a buffer for sending data; room for 2-byte address + 32 bytes data
	if (memAddress > 0xFFFF) {
		eepromDeviceAddress++;							// For memory addresses above 0xFFFF, add 1 to the eeprom device address
	}
	memAccessBuffer[0] = (memAddress >> 8) & 0xFF;		// High byte of memory address gets transmitted first...
	memAccessBuffer[1] = memAddress & 0xFF;				// ...low byte of memory address next
	for (int i = 0; i < length; i++){
		memAccessBuffer[i + 2] = _bytesToWrite[i];		// Move rest of data into buffer
	}
	Wire.beginTransmission(eepromDeviceAddress);		
	Wire.write(memAccessBuffer, length + 2);			// Send the data over i2c
	Wire.endTransmission();
}
void eepromRead(long memAddress, int length, byte _bytesReceived[]) {
	int eepromDeviceAddress = 0x50;						// 7-bit i2c address for Atmel AT24CM01; 8-bit version is 0xA0
	byte memAccessBuffer[2];
	memAccessBuffer[0] = (memAddress >> 8) & 0xFF;		// High byte of memory address goes first...
	memAccessBuffer[1] = memAddress & 0xFF;				// ...low byte of memory address next
	if (memAddress > 0xFFFF) {
		eepromDeviceAddress++;							// For memory addresses above 0xFFFF, add 1 to the eeprom device address
	}
	Wire.beginTransmission(eepromDeviceAddress);	
	Wire.write(memAccessBuffer , 2);					// "Dummy write" of address bytes to position the pointer
	Wire.endTransmission();
	Wire.beginTransmission(eepromDeviceAddress);
	Wire.requestFrom(eepromDeviceAddress, length);		// Now request the appropriate number of bytes to read	
	int bytePos = 0;
	while (bytePos < length)	{
		_bytesReceived[bytePos] = Wire.read();
		bytePos++;
	}
}
