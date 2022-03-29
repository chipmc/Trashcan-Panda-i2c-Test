/*
 * Project Trashcan-Panda-i2c-Test
 * Description: Simple code to figure out what is going on with the trashcan panda and i2c
 * Author: Chip McClelland
 * Date:3-13-2022
 */


// Included Libraries
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "LIS3DH.h"
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library


// Pin Constants - Boron Carrier Board v1.x
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor - on the carrier board - inside the enclosure
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int disableModule = D2;                       // Pin to shut down the device - active low
const int intPin =        D3;                       // Hardware interrupt - poliarity set in the library

// Prototypes and System Mode calls
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
LIS3DHI2C accel(Wire, 1, intPin);                   // Initialize the accelerometer in i2c mode
SFEVL53L1X distanceSensor;                          // Initialize the TOF sensor - no interrupts
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library


// For monitoring / debugging, you can uncomment the next line
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Once intialized, we will give some values
bool sensorOnlineTOF = false;
bool sensorOnlineLIS3DH = false;
bool carrierOnline = false;
unsigned long lastSampleTimestamp;
unsigned long sampleRateMillis = 2000;
const int FRAMversionNumber = 42;                    // Note, this will reset the FRAM from what is in the visitation counters
bool powerCycleOnce = true;                        // One time, we need to test the ability to power down the board, ensure the i2c bus is working and then power back up

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};


// setup() runs once, when the device is first turned on.
void setup() {

  Wire.begin();
  pinMode(intPin,INPUT);
  pinMode(disableModule,OUTPUT);
  digitalWrite(disableModule,LOW);


  delay(1000);


  i2cScan();

	// Initialize LIS3DH sensor
	LIS3DHConfig config;
	config.setAccelMode(LIS3DH::RATE_100_HZ);

	bool setupSuccess = accel.setup(config);
  if (setupSuccess) {
    Log.info("LIS3DH Initialized successfully");
    sensorOnlineLIS3DH = true;
  }
  else Log.info("LIS3DH failed initialization");

  // Initialize the VL53L0X sensor
  if (distanceSensor.begin() == 0) {
    Log.info("TOF Sensor Initialized successfully");
    sensorOnlineTOF = true;
  }
  else Log.info("TOF Sensor failed initialization");

  // Initilize the FRAM Module
  fram.begin();                                                        // Initialize the FRAM module
  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);                            // Load the FRAM memory map version into a variable for comparison
  if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
    Log.info("FRAM Module initializing version %i",FRAMversionNumber);
    fram.erase();                                                      // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
    if (tempVersion != FRAMversionNumber) {
      carrierOnline = false;
      Log.info("FRAM test failed, expected %i and found %i",FRAMversionNumber, tempVersion);
    }
    else carrierOnline = true;
  }
  else {
    carrierOnline = true;
    Log.info("FRAM Module loaded successfully");
  }

  // Watchdog Timer and Real Time Clock Initialization
  ab1805.withFOUT(D8).setup();                                         // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);                         // Enable watchdog
  if (ab1805.detectChip()) {
    Log.info("AB1805 detected");
  }
  else {
    Log.info("AB1805 Failed to Initialize");
    if (carrierOnline) carrierOnline = false;
  }

  if (sensorOnlineLIS3DH && sensorOnlineTOF && carrierOnline) {
    Log.info("Initialization Complete all devices on-line");
    delay(500);
    Log.info("Starting a running sample run");
    Log.info("*****************************");
  }
  else {
    Log.info("Errors in Initialization - Stopping");
    while(1);
  }

}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

if (millis() - lastSampleTimestamp > sampleRateMillis) {
  lastSampleTimestamp = millis();

  if (sensorOnlineTOF) {
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();
    Log.info("Distance measured = %i",distance);
  }

  if (sensorOnlineLIS3DH) {
    LIS3DHSample sample;
		if (accel.getSample(sample)) {
			Log.info("%d,%d,%d", sample.x, sample.y, sample.z);
		}
		else {
			Serial.println("no sample");
		}
  }
}

if (powerCycleOnce) {
  powerCycleOnce = false;                                   // Only do this test once
  Log.info("Devices on line with power");
  i2cScan();
  Log.info("Powering down the sensor board");
  digitalWrite(disableModule,true);                         // Powers down the board
  delay(1000);                                              // Discharge the bypass caps
  Log.info("Devices on-line without power");
  i2cScan();
  if (ab1805.detectChip()) Log.info("AB1805 detected");
  Log.info("Bringing sensors back on-line");
  digitalWrite(disableModule,false);
  Log.info("Devices on-line with power restored");
  i2cScan();
  // LIS3DH Sensor
  LIS3DHConfig config;
  config.setAccelMode(LIS3DH::RATE_100_HZ);

	bool setupSuccess = accel.setup(config);
  if (setupSuccess) {
    Log.info("LIS3DH Initialized successfully");
    sensorOnlineLIS3DH = true;
  }
  else Log.info("LIS3DH failed initialization");

  // Initialize the VL53L0X sensor
  if (distanceSensor.begin() == 0) {
    Log.info("TOF Sensor Initialized successfully");
    sensorOnlineTOF = true;
  }
  else Log.info("TOF Sensor failed initialization");
  if (sensorOnlineLIS3DH && sensorOnlineTOF) {
    Log.info("Power on test complete - measuring resumed");
  }
  else {
    Log.info("Power on test failed - stopping");
    while(1);
  }

}

}

bool i2cScan() {                                            // Scan the i2c bus and publish the list of devices found
	byte error, address;
	int nDevices = 0;
  char resultStr[128];
  strncpy(resultStr,"i2c device(s) found at: ",sizeof(resultStr));

	for(address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
      char tempString[4];
      snprintf(tempString, sizeof(tempString), "%02X ",address);
      strncat(resultStr,tempString,4);
			nDevices++;
      if (nDevices == 9) break;                    // All we have space to report in resultStr
		}

		else if (error==4) {
      snprintf(resultStr,sizeof(resultStr),"Unknown error at address %02X", address);
      Log.info(resultStr);
      return 0;
		}
	}

	if (nDevices == 0) {
    snprintf(resultStr,sizeof(resultStr),"No I2C devices found");
    Log.info(resultStr);
    return 0;
  }

  Log.info(resultStr);
  return 1;
}