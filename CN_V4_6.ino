

#include <PID_v1.h>

#include <LOPCLibrary.h>
#include <math.h> //needed for log in P to Z calculation

/*
 * CN Operating Code
 * Purpose: To demonstrate the functions provided by the EEProm Library.
 *
 * 
 * Date: Feb 11, 2017
 * 
 * April 2018: Modify for second sheath flow pump
 * V4.0 August 2018: Modify output format to be identical to SD format
 * V4.1 January 2019: add Xdata input from heated inlet and output to LOPC or iMet
 * V4.2 Move OPC serial routine to check serial, with flag for string received
 *      Alternate which temps are measured on each cycle to save time - otherwise we miss OPC serial.
 * V4.3 February 2019 Change Saturator heater control to be proportional PWM driven.
 * V4.4 Apri; 2019 Add an extra saturator/altitude temperature step an interactive control of flow and 
 *            Temperature set points.  Added EEPROM saving of flow setpoints.  Added GPS altitude triggered
 *            shutdown before landing.
 * V4.5 April 2019: Use iMet GPS position and/or pressure as a back up for CN set point changes and shutdown.
 * V4.6 June 2020: Setup the MetOne OPC to report 2s data at startup. 
 *                 Add a minimun time before shutdown to prevent early shutdown.
 */

String Version = "V4_6";
//Altitude CN set points
#define ALT1 22000.0
#define ALT2 26000.0
#define ALT3 30000.0
#define ALT4 35000.0

#define SHUTDOWN_ALT 3000  // Altitude threshold to shutdown on descent in meters
#define MIN_FLIGHT_TIME 90*60*1000 // Minimum on time before we will shutdown for end of flight -  in milliseconds.


#define T1 35.0  //Cha
#define T2 35.0
#define T3 35.0
#define T4 40.0
#define T5 45.0
#define HYSTERESIS 0.3

#define IPUMP1_LIMIT 500.0   //Pump current limit in mA
#define IPUMP2_LIMIT 500.0   //Pump current limit in mA


// Serial Port constants
#define DEBUG_SERIAL Serial
#define MET1OPCSERIAL Serial1
#define GPSSerial Serial4
#define IMET_SERIAL Serial2
#define HEATER_SERIAL Serial3

//EEPROM Addresses
#define EEPROM_FLAG  30 //value to say there is valid pump data in eeprom
#define EEPROM_PUMP1 40 //pump 1 BEMF (float)
#define EEPROM_PUMP2 50 //pump 2 BEMF (float)

TinyGPSPlus gps;
boolean usingInterrupt = false;
boolean gpsStatus[] = {false, false, false, false, false, false, false};

double TempCN;
float TempPump1;
float TempPump2;
float TempCN2;
float TempPCB;
float TempIce;
float TempCase;

String OPC_Buff = "";
String OutputString = "";
String file;
String DEBUG_Buff = "";
bool Heater;
bool HeaterShutdown = false;
long Frame = 0;
long ElapsedTime = 0;

float IPump1;
float IPump2;
float IHeater1;
float IHeater2;
float IDetector;
float VDetector;
float VBat;
float VTeensy;
float VPHA;


/*Back EMF Control Variables */
int backEMF1 = 0;
int backEMF2 = 0;
float BEMF1_V = 0;
float BEMF2_V = 0;

/*default BEMF Settings */
float BEMF1_SP = 8.0; //Set point for sheath pump ~ 1.32 SLM (Boulder)
float BEMF2_SP = 8.0; //Set point for exhaust pump ~ 2.50 SLM (Boulder)
         
float error1 = 10.0;
float error2 = 10.0;
float Kp = 20.0;
int BEMF1_pwm = 0;
int BEMF2_pwm = 0;

/*Heater Proportional Control Variables*/
float HeaterError = 0.0;
float HeaterKp = 1500.0;
float HeaterSetPoint = T1;
int HeaterPWM = 500;

/*Shutdwon before landing variables */
float AltLimit = 3000.0;  // Altitude to shutdown at on descent. 
bool InFlight = false; 
float alt_to_use = 0.0; //Variable to store which ever altitude we are using
uint8_t alt_status = 4; //status byte to identify which data source for altitude

//Define Variables we'll be connecting to
double CNSetpoint = T1, SaturatorOutput;

//Specify the links and initial tuning parameters
PID CN_PID(&TempCN, &SaturatorOutput, &CNSetpoint,800,20,500, DIRECT);



String IMET_Buff;
float iMet_p = 0.0;
float iMet_t = 0.0;
float iMet_u = 0.0;
float iMet_lat = 0.0;
float iMet_lon = 0.0;
float iMet_alt = 0.0;
float iMet_sats = 0.0;

int OPC_Time;
int OPC_300;
int OPC_500;
int OPC_700;
int OPC_1000;
int OPC_2000;
int OPC_2500;
int OPC_3000;
int OPC_5000;

int T_UpStream; 
int T_Middle;
int T_DownStream;
int HI_V_Battery;
int Valve_State;

bool OPCStringReady = false;
bool XDataInputAvailable = false;
bool HI_Data_Available = false;
String XDataInput;
String header = "; Date, Time, Altitude, Latitude, Longitude, iMET_Pressure [hPa], iMet_Temperature [C], iMet_RH [%], iMet_Lat, iMet_lon, iMet_alt, iMet_Sats, Alt_in_use, T_CN [C], T_IceJacket [C], T_Pump1 [C], T_Pump2 [C], T_PCB [C], T_Case [C], I_Pump1 [A], I_Pump2 [A], I_Det [mA], V_det, V_PHA, V_Bat, BEMF_1, Pump1_PWM, BEMF_2, Pump2_PWM, CN_Heater_SetPoint, CN_Heater_Output, OPC_Elapsed_Time, OPC_300nm, OPC_500nm, OPC_700nm, OPC_1000nm, OPC_2000nm, OPC_2500nm, OPC_3000nm, OPC_5000nm' ";

bool Plotter = false;

LOPCLibrary CN(13);  //Creates an instance of the OPC/CN Library

void setup() {
   pinMode(PULSE_LED, OUTPUT);
   digitalWrite(PULSE_LED, HIGH);  
   DEBUG_SERIAL.begin(115200);
   delay(100);
   DEBUG_SERIAL.println("CN Mainboard");
   DEBUG_SERIAL.println(Version);
   
   CN.SetUp();  //setup the board
   CN.configure_memory_table(); //Setup the custom thermistor specs
   CN.ConfigureChannels(); //Setup the LTC2983 Channels

  //GPS Serial Setup
   GPSSERIAL.begin(9600);
  
   //Configure UBlox to only send the NMEA strings we want
   byte settingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; 
   configureUblox(settingsArray); 

   HEATER_SERIAL.begin(9600);

   /*IMET Setup */
   DEBUG_SERIAL.println("Configuring iMet");
   IMET_SERIAL.begin(9600); 
   delay(1000);
   IMET_SERIAL.println("ptux=on");
   delay(500);
   IMET_SERIAL.println("gps=on");
   delay(500);
   IMET_SERIAL.println("save");
   

   digitalWrite(PULSE_LED, HIGH);
   
   //OPC Serial port setup
  MET1OPCSERIAL.begin(9600, SERIAL_8N1_RXINV_TXINV);  //Serial is inverted for MetOne 9722 OPC
  delay(500);
  MET1OPCSERIAL.setTimeout(2000); //Set the serial timout to 2s

  digitalWrite(PHA_POWER, HIGH); //turn on the 9722 OPC

  int type = CN.InstrumentType();
  int serial = CN.SerialNumber();
  int filenum = CN.FileNumber();
  DEBUG_SERIAL.println("Startup");
  DEBUG_SERIAL.println(Version);
  DEBUG_SERIAL.print("Type: ");
  DEBUG_SERIAL.println(type);
  DEBUG_SERIAL.print("Serial: ");
  DEBUG_SERIAL.println(serial);
  DEBUG_SERIAL.print("File Number: ");
  DEBUG_SERIAL.println(filenum);
  
    //This creates a file name that can be stored on the SD card
  file = CN.CreateFileName();
  DEBUG_SERIAL.print("Filename: ");
  DEBUG_SERIAL.println(file);   

  //This checks to see if the file name has been used.
  if (CN.FileExists(file) == true)
    file = CN.GetNewFileName();
  String Version = "; Firmware Version: 4.6, Main Board Revision: D, OPC Type: Metone 9722 with sheath, Sample Flow Rate: 1.0 LPM, Sheath Flow: 1.5 LPM";
  CN.WriteData(file, Version+"\n");
  CN.WriteData(file, header + "\n"); 

//  delay(2000); //Let the optical head boot up
// MET1OPCSERIAL.write('\n');
// MET1OPCSERIAL.write('\n');
// MET1OPCSERIAL.write('\n');
// MET1OPCSERIAL.write('H');
// MET1OPCSERIAL.print("T 2"); //set the measurent cadence (s)
// MET1OPCSERIAL.write('\n');
// MET1OPCSERIAL.write('\n');
  analogReadResolution(12);
  analogReference(EXTERNAL);
  analogReadAveraging(32);

  /*Check to see if we have valid pump set points in EEPROM and read if we do */
  if (EEPROM.read(EEPROM_FLAG) == 0x01)
  {
    EEPROM.get(EEPROM_PUMP1, BEMF1_SP);
    DEBUG_SERIAL.print("Saved Sheath Pump Setting loaded: ");
    DEBUG_SERIAL.println(BEMF1_SP);
    EEPROM.get(EEPROM_PUMP2, BEMF2_SP);
    DEBUG_SERIAL.print("Saved Exhaust Pump Setting loaded: ");
    DEBUG_SERIAL.println(BEMF2_SP);
  }
  else
  {
    DEBUG_SERIAL.println("Using default pump settings");
  }
  
  analogWriteResolution(10);
  analogWriteFrequency(PUMP1_PWR, 1000);
  analogWriteFrequency(PUMP2_PWR, 1000);
  analogWriteFrequency(HEATER1, 1000);

  
  CN_PID.SetMode(AUTOMATIC);
  CN_PID.SetOutputLimits(0, 1000);
  CN_PID.SetSampleTime(1000);
  
  digitalWrite(PULSE_LED, LOW);
  ReadHK();
  AdjustPumps();
  DEBUG_SERIAL.println(header);
}

void loop() {      
   checkForSerial();
      
   if (OPCStringReady) {
      parseOPC();  
      checkForSerial();
          
     /* Get temperatures */
      TempCN = CN.MeasureLTC2983(HEATER1_THERM);      // Ch 8: Thermistor 44006 10K@25C
      checkForSerial();
//      if(TempCN < -200.0)
//        ResetLTC2983();

      if(TempCN > -40.0) //don't adjust stack temp if TempCN in invalid
        CN_PID.Compute();
      
        if (Frame%4 ==0)
        {
        TempPump1 = CN.MeasureLTC2983(PUMP1_THERM);     // Ch 6: Thermistor 44006 10K@25C
        checkForSerial();
//        if(TempPump1 < -200.0)
//          ResetLTC2983();
        }
        
        if (Frame%4 ==1)
        {
        TempPump2 = CN.MeasureLTC2983(PUMP2_THERM);     // Ch 6: Thermistor 44006 10K@25C
        checkForSerial();
//      if(TempPump2 < -200.0)
//          ResetLTC2983();
        }
        if (Frame%4 ==2)
        {
        TempPCB = CN.MeasureLTC2983(BOARD_THERM);
        checkForSerial();
 //       if(TempPCB < -200.0)
//          ResetLTC2983();
        }

        if (Frame%4 ==3)
        {
        TempIce = CN.MeasureLTC2983(HEATER2_THERM);
      checkForSerial();
//      if(TempIce < -200.0)
//        ResetLTC2983();
        }

      checkForSerial();
      
      ReadHK();
      checkForSerial();
      if (gps.altitude.age() < 2000) //if the CNC GPS has a solution, use that
        {
          alt_to_use = gps.altitude.meters();
          alt_status = 0;
        }
      else if (iMet_sats > 4.0)
      {
        alt_to_use = iMet_alt;
        alt_status = 1;
        DEBUG_SERIAL.print("Alt_to_use from iMet GPS: "); DEBUG_SERIAL.println(alt_to_use);
      }
      else if (iMet_p > 0.1)
      {
        alt_to_use = (-6.167 * log(iMet_p) + 43.263)*1000.0; //Z to P fit from iMet data on MM1028
        alt_status = 2;
      }
      else
        alt_status = 3;  
        
      HeaterSetPoint = adjustCNHeater(alt_to_use);
      checkForSerial();
      //printGPS();   
      StringGPS();
      StringTemps();
      StringOPC();
     
      if (!Plotter)
      {
        DEBUG_SERIAL.println(OutputString);
        DEBUG_SERIAL.send_now();  //flush
      }
      if (Plotter)
      {
        DEBUG_SERIAL.print(OPC_300/33.0); DEBUG_SERIAL.print(",");
        DEBUG_SERIAL.print(OPC_500/33.0); DEBUG_SERIAL.print(",");
        DEBUG_SERIAL.print(OPC_700/33.0); DEBUG_SERIAL.print(",");
        DEBUG_SERIAL.println(OPC_1000/33.0); 
      }
      OutputString += "\n";
      checkForSerial();
      CN.WriteData(file, OutputString); 
      checkForSerial();
      SendXData(IMET_SERIAL); 
      OPC_Buff = "";  //Clear Strings
      OutputString = "";  
      checkForSerial();
      ElapsedTime = millis();  //Time since instrument start in milliseconds
      Frame++;
      
      if(((gps.altitude.meters() > (AltLimit + 20.0)) && (gps.altitude.meters() > 0) )|| ((iMet_alt > (AltLimit + 20.0)) && (iMet_alt > 0))) // if we go over the thershold altitude we are in flight
        {
        if(millis() > MIN_FLIGHT_TIME) // we are only in flight after MinFlightTime seconds have elapsed
            InFlight = true;
        DEBUG_SERIAL.println("In flight altitude reached!");
        }

      if((gps.altitude.meters() < (AltLimit - 20.0) && InFlight && (gps.altitude.meters() > 0)) || (iMet_alt < (AltLimit - 20.0) && InFlight && (iMet_alt > 0)) ) // once we are back down below altitude, shut down (groundmode)
        {
        DEBUG_SERIAL.println("Entering Ground Mode");
        CN.WriteData(file, "End of Flight - Entering Ground Mode\n");  // Write a data line to SD card
        GroundMode(); 
        }
       
       if(VBat < 14.0)
        {
          DEBUG_SERIAL.println("Low Battery Voltage Entering Ground Mode");
         CN.WriteData(file, "Low Battery Voltage - Entering Ground Mode\n");  // Write a data line to SD card
          GroundMode();
        }         
     //printGPS();
     //printHK();
     AdjustPumps(); 
   }

    checkForSerial();
     
     
  
}

/*
 * Debug routine for GPS reciever, dumps GPS data to console
 */

void printGPS()
{
 
 // DEBUG_SERIAL.print(" Date: "); DEBUG_SERIAL.print(gps.date.year()); DEBUG_SERIAL.print(gps.date.month()); DEBUG_SERIAL.print(gps.date.day());
  DEBUG_SERIAL.print(" Time: "); DEBUG_SERIAL.print(gps.time.value());
    if (gps.altitude.isValid()) {
      
       DEBUG_SERIAL.print(" LAT: ");  DEBUG_SERIAL.print(gps.location.lat(), 9);
       DEBUG_SERIAL.print(" LONG: "); DEBUG_SERIAL.print(gps.location.lng(), 9);
      DEBUG_SERIAL.print(" ALT: ");  DEBUG_SERIAL.print(gps.altitude.meters());
    }
    DEBUG_SERIAL.println();
    if(!gps.altitude.isValid())
    digitalWrite(PULSE_LED, LOW);
}

/*
 * Creates a string with formatted GPS data for the data file
 */

void StringGPS()
{
    OutputString += String(gps.date.value()); 
    OutputString += ", ";
    
    OutputString += String(gps.time.value());
    OutputString += ", ";

    OutputString += gps.altitude.meters();
    OutputString += ", ";

    OutputString += String(gps.location.lat(),6);
    OutputString += ", ";
    
    OutputString += String(gps.location.lng(),6);
    OutputString += ", ";
    

    if (gps.altitude.isValid()) 
      digitalWrite(PULSE_LED, HIGH);
    if(!gps.altitude.isValid())
      digitalWrite(PULSE_LED, LOW);
}

void StringTemps()
{
  
  OutputString += iMet_p; 
  OutputString += ", ";
  OutputString += iMet_t; 
  OutputString += ", "; 
  OutputString += iMet_u; 
  OutputString += ", "; 
  OutputString += String(iMet_lat,6); 
  OutputString += ", ";  
  OutputString += String(iMet_lon,6); 
  OutputString += ", "; 
  OutputString += iMet_alt; 
  OutputString += ", "; 
  OutputString += iMet_sats; 
  OutputString += ", "; 
  OutputString += alt_to_use; 
  OutputString += ", "; 
  OutputString += TempCN;
  OutputString += ","; 
  OutputString += TempIce;
  OutputString += ","; 
  OutputString += TempPump1; 
  OutputString += ", "; 
  OutputString += TempPump2; 
  OutputString += ", ";  
  OutputString += TempPCB; 
  OutputString += ", "; 
  OutputString += TempCase; 
  OutputString += ", "; 
  OutputString += IPump1; 
  OutputString += ", "; 
  OutputString += IPump2;  
  OutputString += ", "; 
  OutputString += IDetector; 
  OutputString += ", "; 
  OutputString += VDetector;  
  OutputString += ", "; 
  OutputString += VPHA;  
  OutputString += ", "; 
  OutputString += VBat;  
  OutputString += ", "; 
  OutputString += BEMF1_V;  
  OutputString += ", "; 
  OutputString += BEMF1_pwm;  
  OutputString += ", "; 
  OutputString += BEMF2_V;  
  OutputString += ", "; 
  OutputString += BEMF2_pwm;  
  OutputString += ","; 
  OutputString += HeaterSetPoint; 
  OutputString += ","; 
  OutputString += SaturatorOutput;
  OutputString += ","; 

if (HI_Data_Available)
{
  OutputString += T_UpStream;
  OutputString += ","; 
  OutputString += T_Middle;
  OutputString += ",";  
  OutputString += T_DownStream;
  OutputString += ","; 
  OutputString += HI_V_Battery;
  OutputString += ","; 
  OutputString += Valve_State;
  OutputString += ","; 
  HI_Data_Available = false;
}
  
}

void StringOPC()
{
  OutputString += OPC_Time; 
  OutputString += ", ";
  OutputString += OPC_300; 
  OutputString += ", ";
  OutputString += OPC_500; 
  OutputString += ", ";
  OutputString += OPC_700; 
  OutputString += ", ";
  OutputString += OPC_1000; 
  OutputString += ", ";
  OutputString += OPC_2000; 
  OutputString += ", ";
  OutputString += OPC_2500; 
  OutputString += ", ";
  OutputString += OPC_3000; 
  OutputString += ", ";
  OutputString += OPC_5000; 
  OutputString += ", ";
  
}
void ResetLTC2983(void)
{
  int Done = 0;
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  while(!Done)
  {
    Done = digitalRead(INTERUPT);
  }
  CN.configure_memory_table(); //Setup the custom thermistor specs
   
  //DEBUG_SERIAL.println("LTC2983 has been reset");
  CN.ConfigureChannels();
  //DEBUG_SERIAL.println("Part has been configured");  
}

float adjustCNHeater(float alt)
{
  if (alt > 3000.0)
  {

  if(alt <= ALT1)
    CNSetpoint = T1;
  if(alt > ALT1 && alt <= ALT2)
    CNSetpoint = T2;
  if(alt > ALT2 && alt <= ALT3)
    CNSetpoint = T3;
  if(alt > ALT3 && alt <= ALT4)
    CNSetpoint = T4;
  if(alt > ALT4)
    CNSetpoint = T5;
  }
  CN_PID.Compute();
  analogWrite(HEATER1, int(SaturatorOutput));
  
  return CNSetpoint;
}

void ReadHK(void)
{
 long I1_Bits = 0;
 long I2_Bits = 0;
 int i = 0;  
 int StartTime = millis();
  
 while (millis() - StartTime < 10) //Average pump current for > 2ms to make sure we get a full pwm cycle
 {
 
    I1_Bits += analogRead(I_PUMP1);
    I2_Bits += analogRead(I_PUMP2);
    i++;
 }
    
  IPump1 = (I1_Bits/i)/4095.0 * 3000;
  IPump2 = (I2_Bits/i)/4095.0 * 3000;
/*
  if(IPump1 > IPUMP1_LIMIT)
  {
    DEBUG_SERIAL.println("Pump 1 Overcurrent! Shutting down");
    GroundMode();
  }
  if(IPump2 > IPUMP2_LIMIT)
  {
    DEBUG_SERIAL.println("Pump 2 Overcurrent! Shutting down");
    GroundMode();
  }
  */
  IHeater1 = analogRead(HEATER1_I)/1.058;
  IHeater2 = analogRead(HEATER2_I)/1.058;
  IDetector = analogRead(PHA_I)/1.058;
  VDetector = analogRead(PHA_12V_V)*3.0/4095.0*5.993;
  VPHA = analogRead(PHA_3V3_V)*3.0/4095.0*2.0; 
  VBat = analogRead(BATTERY_V)*3.0/4095.0 *6.772;
  VTeensy = analogRead(TEENSY_3V3)*3.0/4095.0*2.0;
}

void printHK()
{
  DEBUG_SERIAL.print("CN Stack Temperature [C]: ");
  DEBUG_SERIAL.println(TempCN);
  DEBUG_SERIAL.print("Pump1 [C]: ");
  DEBUG_SERIAL.println(TempPump1);
  DEBUG_SERIAL.print("Pump2 [C]: ");
  DEBUG_SERIAL.println(TempPump2);
  DEBUG_SERIAL.print("Main Board Temperature [C]: ");
  DEBUG_SERIAL.println(TempPCB);
  DEBUG_SERIAL.print("Ice Jacket [C]: ");
  DEBUG_SERIAL.println(TempIce);
  DEBUG_SERIAL.print("I Pump1 [mA]: ");
  DEBUG_SERIAL.println(IPump1);
  DEBUG_SERIAL.print("I Pump2 [mA]: ");
  DEBUG_SERIAL.println(IPump2);
  DEBUG_SERIAL.print("V Battery [V]: ");
  DEBUG_SERIAL.println(VBat);
  DEBUG_SERIAL.print("V Teensy [V]: ");
  DEBUG_SERIAL.println(VTeensy);
  DEBUG_SERIAL.print("V OPC [V]: ");
  DEBUG_SERIAL.println(VDetector);
  DEBUG_SERIAL.print("Motor 1 PWM: ");
  DEBUG_SERIAL.print(BEMF1_pwm);
  DEBUG_SERIAL.print(" BEMF: ");
  DEBUG_SERIAL.println(BEMF1_V);
  DEBUG_SERIAL.print("Motor 2 PWM: ");
  DEBUG_SERIAL.print(BEMF2_pwm);
  DEBUG_SERIAL.print(" BEMF: ");
  DEBUG_SERIAL.println(BEMF2_V);
}


void AdjustPumps()
{
  int i = 0;
  int MaxPWM = int(12.0/VBat*1023.0); //This is the maximum PWM frequency we should apply to the pumps to keep the effective voltage below 12
  analogWrite(PUMP1_PWR, 0); //Turn off Pump1
  delayMicroseconds(500); //Hold off for spike to collapse
  
  for(i = 0; i < 32 ; i++)
    backEMF1 += analogRead(PUMP1_BEMF); 

  BEMF1_V = VBat - backEMF1/(4095.0*32.0)*18.0;
  error1 = BEMF1_V - BEMF1_SP;
  BEMF1_pwm = int(BEMF1_pwm - error1*Kp);
  if (BEMF1_pwm > MaxPWM)
    BEMF1_pwm = MaxPWM;
  analogWrite(PUMP1_PWR, BEMF1_pwm);
  delay(10);
  
  analogWrite(PUMP2_PWR, 0); //Turn off Pump1
  delayMicroseconds(500); //Hold off for spike to collapse
  
  for(i = 0; i < 32 ; i++)
    backEMF2 += analogRead(PUMP2_BEMF); 

  BEMF2_V = VBat - backEMF2/(4095.0*32.0)*18.0;
  error2 = BEMF2_V - BEMF2_SP;
  BEMF2_pwm = int(BEMF2_pwm - error2*Kp);
  if (BEMF2_pwm > MaxPWM)
    BEMF2_pwm = MaxPWM;
  analogWrite(PUMP2_PWR, BEMF2_pwm);

  backEMF1 = 0;
  backEMF2 = 0;
}

void GroundMode(void)
{

  analogWrite(PUMP1_PWR, 0); //turn off pump 1
  analogWrite(PUMP2_PWR, 0); // turn off pump 2
  analogWrite(HEATER1, 0); // turn off heater
  digitalWrite(MFS_POWER, LOW); // turn off Mass Flow Meter
  digitalWrite(PHA_POWER, LOW); // turn off OPC
  
  while(1);
}


void parseOPC() {  
  char * strtokIndx; // this is used by strtok() as an index
  char OPCArray[64];
  //DEBUG_SERIAL.println(OPC_Buff);
  
  OPC_Buff.toCharArray(OPCArray, 64);


  strtokIndx = strtok(OPCArray,",");      // get the first part - the timestamp
  OPC_Time = atoi(strtokIndx); 
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_300 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_500 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_700 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_1000 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_2000 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_2500 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_3000 = atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_5000 = atoi(strtokIndx);     // convert this part to an integer

  OPCStringReady = false;
  OPC_Buff = "";
 
}

void  SendXData(Stream &port){
  //Instrument ID "41"  Again, just made this up
  String header = "xdata=4102";
  IMET_SERIAL.print(header); 
  /* Xdata format
   *  xdata=4102  - instriment ID and daisy chain number
   *  AAAA - OPC_300 (2 bytes, 0 - 65535)
   *  BBBB - OPC_500 (2 bytes, 0 - 65535)
   *  CCCC - OPC_700 (2 bytes, 0 - 65535)
   *  DDDD - OPC_1000 (2 bytes, 0 - 65535)
   *  EEEE - OPC_2000 (2 bytes, 0 -65535)
   *  FFFF - OPC_2500 (2 bytes, 0 -65535)
   *  GGGG - OPC_3000 (2 bytes, 0 -65535)
   *  HHHH - OPC_5000 (2 bytes, 0 -65535)
   *  II - Pump 1 PWM drive (%) (0 - 100)
   *  JJ - Pump 2 PWM Drive (%) (0 - 100)
   *  KK - Pump 1 Current mA  (0 - 255mA)
   *  LL - Pump 2 Current mA  (0 - 255mA) 
   *  MM - Battery Voltage - voltage multiplied by 10 then converted to unsigned short (0 - 25.5V)
   *  NN - Saturator Temperature degress C /100 = (0 - 655.53 C)
   *  OO - Ice Jacket Temp C + 100.0  (-100 - 155C)
   *  PP - PCB Temp (0 - 255)
   *  QQ - Pump1 Temp (-100 - 125)
   *  RR - Pump2 Temp (-100 - 125)
   *  SS - Satruator output %
   *  TT - Altitude status byte
   */
   
  printFixedHex(port, OPC_300,2); //300nm Bin
  printFixedHex(port, OPC_500,2); //500nm Bin
  printFixedHex(port, OPC_700,2); //700nm Bin
  printFixedHex(port, OPC_1000,2); //1000nm Bin
  printFixedHex(port, OPC_2000,2); //2000nm Bin
  printFixedHex(port, OPC_2500,2); //2500nm Bin
  printFixedHex(port, OPC_3000,2); //3000nm Bin
  printFixedHex(port, OPC_5000,2); //5000nm Bin
  printFixedHex(port, byte(int(float(BEMF1_pwm)/1024.0*100.0)), 1); //Pump1 PWM Drive
  printFixedHex(port, byte(int(float(BEMF2_pwm)/1024.0*100.0)), 1); //Pump2 PWM Drive
  printFixedHex(port, byte(IPump1),1); //Pump1 current in mA
  printFixedHex(port, byte(IPump2),1); //Pump1 current in mA
  printFixedHex(port, byte(VBat*10.0), 1); //Battery voltage *10
  printFixedHex(port, short(TempCN*100.0),2); //Saturator T
  printFixedHex(port, byte(TempIce+ 100.0),1); //Ice Jacket T
  printFixedHex(port, byte(TempPCB+100.0),1); //PCB T
  printFixedHex(port, byte(TempPump1+100.0),1); //Pump 1 Temp
  printFixedHex(port, byte(TempPump2+100.0),1); //Pump 2 Temp
  printFixedHex(port, byte(int(float(SaturatorOutput)/1024.0*100.0)),1); //Saturator heater output
  printFixedHex(port, byte(alt_status),1); //Status byte for altitude source
  if(XDataInputAvailable) 
  {
   printFixedHex(port, byte(T_UpStream),1);
   printFixedHex(port, byte(T_Middle),1);
   printFixedHex(port, byte(T_DownStream),1);
   printFixedHex(port, byte(HI_V_Battery),1);
   printFixedHex(port, byte(Valve_State),1);    
   XDataInputAvailable = false;
   HI_Data_Available = true;
  }  
  
  
    port.write('\r');
    port.write('\n');
  
}

void printFixedHex(Stream &port, unsigned short val, int bytes)
{
  if(bytes == 2)
  {
  if (val < 0x10)
    port.print("0");
  if (val < 0x100)
    port.print("0");
  if (val < 0x1000)
    port.print("0");
  port.print(val, HEX);
  }
  else if (bytes == 1)
  {
     if(val > 0xFF)
    {
      port.print("FF");
      return;
    }
    else if (val < 0x10)
    port.print("0");
  
    port.print(val, HEX);    
  }
}

void checkForSerial(void)
{
   if (GPSSERIAL.available()) {
        gps.encode(GPSSERIAL.read()); //this sends incoming NEMA data to a parser, need to do this regularly
   }

   if(MET1OPCSERIAL.available()){
    char OPC_Char = MET1OPCSERIAL.read();
     if(OPC_Char == '\n')
     {
      OPCStringReady = true;
     } else
     {
      OPC_Buff += OPC_Char;
     }   
  }
   
   if(IMET_SERIAL.available()){
    char IMET_Char = IMET_SERIAL.read();
     HEATER_SERIAL.write(IMET_Char);
     if(IMET_Char == '\n')
     {
      parseIMET();
      IMET_Buff = "";
     } else
     {
      IMET_Buff += IMET_Char;
     }   
  }

  if(HEATER_SERIAL.available()){
    char HEATER_Char = HEATER_SERIAL.read();
     if(HEATER_Char == '\n')
     {
      parseXData();
     } else
     {
      XDataInput += HEATER_Char;
     }   
  }

  if (DEBUG_SERIAL.available()) {
   char DEBUG_Char = DEBUG_SERIAL.read();
   if((DEBUG_Char == '\n') || (DEBUG_Char == '\r'))
   {
    parseCommand(DEBUG_Buff);
    DEBUG_Buff = "";
   } else
   {
    DEBUG_Buff += DEBUG_Char;
   }   
  }
}

void parseXData()
{
  char XDATAArray[128];
  int c = -99;
//DEBUG_SERIAL.print("ParseXData XDATAArray: "); DEBUG_SERIAL.println(XDataInput);
  if (XDataInput.startsWith("xdata=4002")) //we have a CNC string
  {
    (XDataInput.substring(10)).toCharArray(XDATAArray, 128); //drop the first 10 chars ('xdata=4002') and process the rest

    c = sscanf(XDATAArray, "%2hx %2hx %2hx %2hx %2hx", &T_UpStream, &T_Middle, &T_DownStream, &HI_V_Battery, &Valve_State);
    
  }
  if (c == 5)
    XDataInputAvailable = true;
  XDataInput = "";
}

void parseIMET()
{
  char * strtokIndx; // this is used by strtok() as an index
  char IMETArray[64];
  if (IMET_Buff.startsWith("PTUX")) //we have a PTU string
  {
   (IMET_Buff.substring(6)).toCharArray(IMETArray,64);
   strtokIndx = strtok(IMETArray,", ");
   iMet_p = atof(strtokIndx);
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_t = atof(strtokIndx); 
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_u = atof(strtokIndx);  
  }
  else if (IMET_Buff.startsWith("GPS")) //we have a PTU string
  {
   (IMET_Buff.substring(6)).toCharArray(IMETArray,64);
   strtokIndx = strtok(IMETArray,", ");
   iMet_lat = atof(strtokIndx);
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_lon = atof(strtokIndx); 
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_alt = atof(strtokIndx); 
   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   iMet_sats = atof(strtokIndx);   
   IMET_Buff = "";
  }

//  DEBUG_SERIAL.print("iMET Variables: ");
//  DEBUG_SERIAL.print(iMet_p);
//  DEBUG_SERIAL.print(" ");
//  DEBUG_SERIAL.print(iMet_t);
//  DEBUG_SERIAL.print(" ");
//  DEBUG_SERIAL.println(iMet_u);
   
}

void parseCommand(String commandToParse)
{
  /* This is where all the commands are interpreted and is the meat of the controll system
   * so far
   * #stop  - switches to ground mode
   * #header - print a header to the terminal
   * #Exhaust, float - set a new pump speed for the Exhaust (1)
   * #Sheath, float - set a new pump speed fot the Sheath (2)
   * #Saturator, float - set a new saturator temperature
   * #save - write the pump settings to EEPROM
   * #clear - clear the pump settings from EEPROM (this resets pump to hard coded default)
   * #plotter - toggles between serial plotter output and full display
   */
  
  char * strtokIndx; // this is used by strtok() as an index
  char CommandArray[64];
  int int1 = 0;
  float flt1 = 0;
  

  if(commandToParse.startsWith("#stop"))
  {
    DEBUG_SERIAL.println("Switching to Ground Mode");
    DEBUG_Buff = "";
    GroundMode(); //go to idle mode for almost ever
  }
  else if(commandToParse.startsWith("#head"))
  {
     DEBUG_SERIAL.println(header);
     DEBUG_SERIAL.println();
     
  }
  else if(commandToParse.startsWith("#she"))
  {
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     flt1 = atof(strtokIndx);     // convert this part to a float for the temperature set point

     DEBUG_SERIAL.print("Setting Back EMF 1 (Sheath) to: ");
     DEBUG_SERIAL.print(flt1); DEBUG_SERIAL.println("[V]");

     BEMF1_SP = flt1; //Set point for pump 1
     
     commandToParse = "";
  }
  else if(commandToParse.startsWith("#exh"))
  {
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     flt1 = atof(strtokIndx);     // convert this part to a float for the temperature set point

     DEBUG_SERIAL.print("Setting Back EMF 2 (Exhaust) to: ");
     DEBUG_SERIAL.print(flt1); DEBUG_SERIAL.println("[V]");

     BEMF2_SP = flt1; //Set point for pump 1
     
     commandToParse = "";
  }

  else if(commandToParse.startsWith("#sat"))
  {
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     flt1 = atof(strtokIndx);     // convert this part to a float for the temperature set point
     DEBUG_SERIAL.println();
     DEBUG_SERIAL.print("Setting Saturator temperature to: ");
     DEBUG_SERIAL.print(flt1); DEBUG_SERIAL.println("[C]");

     CNSetpoint = flt1; //Set point for Saturator
     
     commandToParse = "";

     return;
  }
  else if(commandToParse.startsWith("#save"))
  {
     DEBUG_SERIAL.println("Writing Pump Settings to EEPROM");
     EEPROM.put(EEPROM_PUMP1, BEMF1_SP); //save pump1 as float
     EEPROM.put(EEPROM_PUMP2, BEMF2_SP); //save pump2 as float
     EEPROM.write(EEPROM_FLAG, 0x01);    //update flag to show valid data in eeprom
     return; 
     
  }
  else if(commandToParse.startsWith("#cle"))
  {
     DEBUG_SERIAL.println("Clearing Pump settings from EEPROM");
     EEPROM.put(EEPROM_PUMP1, 0.0); //save pump1 as float
     EEPROM.put(EEPROM_PUMP2, 0.0); //save pump2 as float
     EEPROM.write(EEPROM_FLAG, 0x00);    //update flag to show valid data in eeprom
  }
  else if(commandToParse.startsWith("#plot"))
  {
     Plotter = !Plotter;
     if(Plotter)
     DEBUG_SERIAL.println("Switching output to Serial Plotter Mode");
     if(!Plotter)
     DEBUG_SERIAL.println("Switching output to Verbose Mode");
  }
  else if(commandToParse.startsWith("#help") || commandToParse.startsWith("#menu"))
  {
     DEBUG_SERIAL.println("Command not recognized, valid commands: ");
     DEBUG_SERIAL.println("#stop - switch to ground mode");
     DEBUG_SERIAL.println("#sheath, float - set sheath pump (pump1) set point to float volts");
     DEBUG_SERIAL.println("#exhaust, float - set exhaust pump (pump2) set point to float volts");
     DEBUG_SERIAL.println("#saturator, float - set saturator set point to float degrees C");
     DEBUG_SERIAL.println("#save, save the pump settings to EEPROM");
     DEBUG_SERIAL.println("#clear, clear the pump settings from EEPROM");
     DEBUG_SERIAL.println("#plotter, toggle data output for Arduino Plotter");
     commandToParse = "";
  }
}

   
void configureUblox(byte *settingsArrayPointer)
{
  byte gpsSetSuccess = 0;
  DEBUG_SERIAL.println("Configuring u-Blox GPS initial state...");

  //Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setNav[2], sizeof(setNav) - 4);

  //Generate the configuration string for Data Rate
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
  calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);

  //Generate the configuration string for Baud Rate
  byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);

  byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
  byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
  byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

  delay(2500);

  while(gpsSetSuccess < 3)
  {
    DEBUG_SERIAL.print("Setting Navigation Mode... ");
    sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
    if (gpsSetSuccess == 5) {
      gpsSetSuccess -= 4;
      setBaud(settingsArrayPointer[4]);
      delay(1500);
      byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
      sendUBX(lowerPortRate, sizeof(lowerPortRate));
      GPSSERIAL.begin(9600);
      delay(2000);      
    }
    if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
    if (gpsSetSuccess == 10) gpsStatus[0] = true;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("Navigation mode configuration failed.");
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3) {
    DEBUG_SERIAL.print("Setting Data Update Rate... ");
    sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function      
    if (gpsSetSuccess == 10) gpsStatus[1] = true;
    if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("Data update mode configuration failed.");
  gpsSetSuccess = 0;


  while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
    DEBUG_SERIAL.print("Deactivating NMEA GLL Messages ");
    sendUBX(setGLL, sizeof(setGLL));
    gpsSetSuccess += getUBX_ACK(&setGLL[2]);
    if (gpsSetSuccess == 10) gpsStatus[2] = true;
    if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("NMEA GLL Message Deactivation Failed!");
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
    DEBUG_SERIAL.print("Deactivating NMEA GSA Messages ");
    sendUBX(setGSA, sizeof(setGSA));
    gpsSetSuccess += getUBX_ACK(&setGSA[2]);
    if (gpsSetSuccess == 10) gpsStatus[3] = true;
    if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("NMEA GSA Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
    DEBUG_SERIAL.print("Deactivating NMEA GSV Messages ");
    sendUBX(setGSV, sizeof(setGSV));
    gpsSetSuccess += getUBX_ACK(&setGSV[2]);
    if (gpsSetSuccess == 10) gpsStatus[4] = true;
    if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("NMEA GSV Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
    DEBUG_SERIAL.print("Deactivating NMEA RMC Messages ");
    sendUBX(setRMC, sizeof(setRMC));
    gpsSetSuccess += getUBX_ACK(&setRMC[2]);
    if (gpsSetSuccess == 10) gpsStatus[5] = true;
    if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("NMEA RMC Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
    DEBUG_SERIAL.print("Deactivating NMEA VTG Messages ");
    sendUBX(setVTG, sizeof(setVTG));
    gpsSetSuccess += getUBX_ACK(&setVTG[2]);
    if (gpsSetSuccess == 10) gpsStatus[6] = true;
    if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) DEBUG_SERIAL.println("NMEA VTG Message Deactivation Failed!");

  gpsSetSuccess = 0;
  if (settingsArrayPointer[4] != 0x25) {
    DEBUG_SERIAL.print("Setting Port Baud Rate... ");
    sendUBX(&setPortRate[0], sizeof(setPortRate));
    setBaud(settingsArrayPointer[4]);
    DEBUG_SERIAL.println("Success!");
    delay(500);
  }
}


void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    GPSSERIAL.write(UBXmsg[i]);
    GPSSERIAL.flush();
  }
  GPSSERIAL.println();
  GPSSERIAL.flush();
}


byte getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (GPSSERIAL.available()) {
      incoming_char = GPSSERIAL.read();
      if (incoming_char == ackPacket[i]) {
        i++;
      }
      else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;
      }
    }
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      DEBUG_SERIAL.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      DEBUG_SERIAL.println("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;
  }
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
//    DEBUG_SERIAL.println("Success!");
//    DEBUG_SERIAL.print("ACK Received! ");
//    printHex(ackPacket, sizeof(ackPacket));
    return 10;
  }
  else {
//    DEBUG_SERIAL.print("ACK Checksum Failure: ");
//    printHex(ackPacket, sizeof(ackPacket));
//    delay(1000);
    return 1;
  }
}

void setBaud(byte baudSetting) {
  if (baudSetting == 0x12) GPSSERIAL.begin(4800);
  if (baudSetting == 0x4B) GPSSERIAL.begin(19200);
  if (baudSetting == 0x96) GPSSERIAL.begin(38400);
  if (baudSetting == 0xE1) GPSSERIAL.begin(57600);
  if (baudSetting == 0xC2) GPSSERIAL.begin(115200);
  if (baudSetting == 0x84) GPSSERIAL.begin(230400);
}
