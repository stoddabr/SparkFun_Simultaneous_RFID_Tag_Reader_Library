/*
  Reading multiple RFID tags, simultaneously!
  By:Brett Stoddard @ Oregon State University OPEnS Lab
  Date: April 27, 2017
  https://github.com/bsstodda/Simultaneous_RFID_Tag_Reader
  Multiple Sensor Read - Ask the reader to tell us the current moisture value.
  When it has 100 values it will find the average value. Using the average 
  value is more accurate and valuable than any single reading
*/

#include <SoftwareSerial.h> //Used for transmitting to the device

SoftwareSerial softSerial(2, 3); //RX, TX

#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano; //Create instance

#define BUZZER1 9
//#define BUZZER1 0 //For testing quietly
#define BUZZER2 10

byte value_list[100][2];
int readings;

void setup()
{
  Serial.begin(115200);

  while (!Serial);
  Serial.println();
  Serial.println("Initializing...");

  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }

  nano.setRegion(REGION_NORTHAMERICA); //Set to North America

  nano.setReadPower(2000); //5.00 dBm. Higher values may cause USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling

  readings = 0;
}

void loop()
{

  if( readings == 100 ) { 
      int total = 0;
      for (byte x=0; x<100; x++){
        total+= value_list[x][1]; //least significant bit
        total+= (value_list[x][0] * 16); //most sig bit
      }
      total /= 100;
      Serial.print( "Average value is: " );
      Serial.println( total );
      while(1) {};
  } //list is full

  byte myEPC[4]; //Most EPCs are 12 bytes
  byte myEPClength;
  byte responseType = 0;

  while (responseType != RESPONSE_SUCCESS)//RESPONSE_IS_TAGFOUND)
  {
    myEPClength = sizeof(myEPC); //Length of EPC is modified each time .readTagEPC is called

    responseType = nano.readTagSensor402(myEPC, myEPClength, 500); //Scan for a new tag up to 500ms
    Serial.println(F("Searching for tag"));
  }

  //Print EPC
  Serial.print(F(" SEN["));
  for (byte x = 1 ; x < 2 ; x++)
  {
    if (myEPC[x] < 0x10) Serial.print(F("0"));
    Serial.print(myEPC[x], HEX);
    Serial.print(F(" "));
  }
  Serial.println(F("]"));

  //add it to list
  for( byte x = 2; x < myEPClength ; x++)
  {
    value_list[readings][x-2] = myEPC[x];
  }
  readings++;
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
