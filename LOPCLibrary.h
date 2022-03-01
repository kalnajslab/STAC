/*
  EEPROMLibrary7.h
  
  Library for . . . 
  *reading/writing important values to flash memory.
  *Creating files that are stored on the SD card
  
  Created by Melissa Mantey
  1.24.17
  Version 3
*/

#ifndef OPCLibrary6_h
#define OPCLibrary6_h

#include "Arduino.h"
#include "EEPROM.h"

#include <SdFat.h>
#include <SdFatConfig.h>

//Teensy 3.6 specific SD card config
#define USE_SDIO 1

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "SPI.h"
//#include "Wire.h"
#include "stdio.h"
#include "math.h"
#include <i2c_t3.h>  //specialized Teensy 3.6 version of arduino i2c library

// LTC2983 Temperature IC Libraries
#include "LTC2983_configuration_constants.h"
#include "LTC2983_support_functions.h"
#include "LTC2983_table_coeffs.h"

// GPS Library
#include <TinyGPS++.h>

// Serial Port constants
#define OPCSERIAL Serial1
#define GPSSERIAL Serial4


//LTC2983 Constants
#define CHIP_SELECT 15
#define RESET 28
#define INTERUPT 27

//Flow Meter Constants
#define MFS_POWER 24

//DIO Constants
#define PUMP1_PWR 5
#define PUMP2_PWR 6
#define PHA_POWER 25
#define HEATER1 22
#define HEATER2 14
#define PULSE_LED 38

//Analog Constants
#define PUMP1_BEMF A3
#define PUMP2_BEMF A9
#define I_PUMP1 A22
#define I_PUMP2 A10
#define BATTERY_V A2
#define PHA_12V_V A20
#define PHA_3V3_V A21
#define PHA_I A18
#define HEATER1_I A17
#define HEATER2_I A16
#define TEENSY_3V3 A7

//Temperature Channels
#define PUMP1_THERM 4
#define PUMP2_THERM 6
#define HEATER1_THERM 8
#define HEATER2_THERM 10
#define BOARD_THERM 12
#define CASE_THERM 14
#define PHA_THERM 16


class LOPCLibrary
{
  public:
    LOPCLibrary(int pin);
    void SetUp();//Configures Teensy to LTC2983
    void ConfigureChannels(); //Configure LTC2983 Channel settings
    float MeasureLTC2983(int channel);//returns the temperature of a given channel in degrees C.
    void printGPS();
    void printTemps();
    int InstrumentType(); //reads/writes the instrument type (1,2 or 3) from EEPROM, error checking as above.
    int SerialNumber(); //reads/writes the instrument serial number from flash and returns it as an int. Returns -1 if error occured
    int FileNumber(); //reads/writes the file number from EEPROM, error checking as above. Returns -1 if error occured
    int IncrementFile(); //increments the file number
    int ErrorCheck(int serial, int type, int file); //checks to see if errors occured. Returns # of errors found.   
    String CreateFileName();
    bool FileExists(String FileName); //Check to make sure the file name doesn't already exist, return False if it doesn't exist, true exist.
    String GetNewFileName(); //would create an alternative filename 'OPxxyyyy.1' using the first available extension.
    bool WriteData(String FileName, String Data); //Open the file, write the data, close the file, return true on success.
    float ReadAnalog(int channel);
    
  private:
    int _pin;
    int _filecount;
    SdFatSdio _SD;

};

#endif
