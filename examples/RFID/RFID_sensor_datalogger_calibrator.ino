/**RFID and analog sensor datalogger**
 * 
 * Hardware: SmartTrac Dogbone Tags, M6E Nano RFID reader sparkfun shield, Arduino UNO, 
 *    sparkfun gps logger shield, YL-69 + YL-38 soil moisture sensor kit
 * Purpose: This code is meant to log information about the RFID tag's values and the 
 *    values of an already calibrated capacitive soil moisture probe periodically.
 *    Ideally, the data collected can be used to calibrate the RFID tags.
 * Author: Brett Stoddard- OSU OPEnS Lab 5-22-17
 * Legal: Open Source
 */

//RFID stuff
//uses SoftwareSerial library to communicate with RFID module over pins 2,3
//uses custom library to control sending commands to the Nano module
#include <SoftwareSerial.h> //Used for transmitting to the device
SoftwareSerial softSerial(2, 3);//RX, TX
#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano;                      //Create instance
#define NUM_READ 100             //number of reads that are averaged out
                                //50 reads give 99% conf intv of .5
int readings;                   //used to keep track of number of reads
byte value_list[NUM_READ][2];

//SD card stuff
//uses the SPI library to select
//uses the SD card library to write to card
#include <SPI.h>
#include <SD.h>
#define chipSelect 10
#define fileName "datalog.txt"

//timer stuff
//uses millis() function to measure elapsed time
unsigned long previousMillis = 0;        // will store last time LED was updated
#define MEASURE_PEROID (1000*60)        //every minute a measurement will be taken

void setup() {
//HW serial setup
  Serial.begin(115200);
  while (!Serial);
  Serial.println();

//RFID setup
  Serial.println(F("Initializing RFID reader..."));
  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    Serial.println(F("Module failed to respond. Please check wiring."));
    while (1); //Freeze!
  }
  Serial.println(F( "Module ready"));
  nano.setRegion(REGION_NORTHAMERICA); //Set to North America
  nano.setReadPower(2000); //5.00 dBm. Higher values may cause USB port to brown out
         //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling
  readings = 0;  

//setup card
  Serial.print(F("Initializing SD card..."));
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("Card initialized."));
}

void loop() {
  String dataString = "";
  unsigned long currentMillis = millis();
  // put your main code here, to run repeatedly:
  if( currentMillis - previousMillis >= MEASURE_PEROID ){ //after input is sent run and output to serial 
    previousMillis = currentMillis;  //restart timer
    Serial.println( Serial.read() ); //dump input
//RFID
    double recent_read = readRFID();
//Cap sensor
    int sensor = analogRead( A0 );
//Serial output
    dataString += String( recent_read , 2);
    Serial.println( dataString );
//SD card output
    if (logData( recent_read , sensor )) // Log the GPS data
      {
        Serial.println(F("Datapoint logged.")); // Print a debug message
      }
      else // If we failed to log GPS
      { // Print an error, don't update lastLog
        Serial.println(F("Failed to log new GPS data."));
      }
  }  
}

//////////RFID functions//////////////////
//read RFID info from a tag, average out NUM_READ reads
double readRFID() {
  while(1){
    if( readings >= NUM_READ ) { 
        int total_ = 0;
        for (byte x=0; x<NUM_READ; x++){    //add up total
          total_ += value_list[x][1];           //least significant bit
          total_ += (value_list[x][0] * 16);    //most sig bit
        }
       readings = 0;
       return double(total_) / NUM_READ ;
    } //list is full
  //read data
    byte myEPC[4]; //from copied code that recorded EPC, this code will not capture EPC info
    byte myEPClength;
    byte responseType = 0;
    while (responseType != RESPONSE_SUCCESS)//RESPONSE_IS_TAGFOUND)
    {
      myEPClength = sizeof(myEPC); //Length of EPC is modified each time .readTagEPC is called
      responseType = nano.readTagSensor402(myEPC, myEPClength, 500); //Scan for a new tag up to 500ms
      Serial.print(F("Searching..."));
      digitalWrite(13,HIGH);
    }
  //add it to list
    for( byte x = 2; x < myEPClength ; x++)
    {
      value_list[readings][x-2] = myEPC[x];
      Serial.print(myEPC[x]);
    }
    Serial.println(", ");
    readings++;
  }
  return 1;
}
//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(softSerial.available()) softSerial.read();
  
  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    softSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

//////////////SD card functions//////////////////////////////////////////////////
byte logData( double RFID_info, int CAP_info )
{
  File logFile = SD.open( fileName, FILE_WRITE ); // Open the log file

  if (logFile)
  { 
    logFile.print( RFID_info , 2 );
    logFile.print(',');
    logFile.print( CAP_info );
    logFile.print(';');
    logFile.close();
    return 1; // Return success
  }

  return 0; // If we failed to open the file, return fail
}
