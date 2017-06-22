/*
 * Reading the RFID tags Sensor Value, and Moisture sensor to calibrate RFID tags. Logging every 10 minutes
 * Hardware: SmartTrac Dogbone Tags, M6E Nano RFID reader on sparkfun board, Arduino UNO
 *          sparkfun gps logger board, ECHO2O EC-TM or 5TM moisture sensor
 * Purpose: To test the the relationship between Tag and actual moisture values
 * Uses code from: tagSEN_EPC_GPS2_SD (for sd card), multiple_SEN_read_list (for RFID), moisture_sensor_EX
 * Dependencies: -SparkFun_UHF_RFID_Reader.h -> from https://github.com/stoddabr/SparkFun_Simultaneous_RFID_Tag_Reader_Library
 *               -SPI -> stock
 *               -SdFat.h -> https://github.com/greiman/SdFat
 *               -SDISerial -> https://github.com/joranbeasley/SDISerial
 *               -Wire -> stock, used this tutorial https://www.arduino.cc/en/Tutorial/MasterReader
 * 
 * Author: Brett Stoddard- OSU OPEnS Lab
 * Legal: Open Source
 * 
 * Moisture sensor Hookup: the WHITE wire goed to 5V. however you could also connect it to a pin and drive power only when you wanted it
 * the RED wire is the DATA_PIN. - you must hook it up to a pin that can process interrupts (see link below)  
 * the remaining "shield" wire must be connected to ground
 * 
 * 
 * info will save to CSV file with 1st column being RFID info, second being moisture sensor, 
 * third being reading #, fourth being frequency
 */
/*/////////////TO DO: figure out the moisture sensor//////////////*/

#include <SoftwareSerial.h>

//RFID stuff
#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano; //create instance 
int currSensorVal;  //sensor data will be 5 bits for 402h SmartTrac tags
uint32_t freq;
#define NUM_READS 50
SoftwareSerial rfidSerial(2,3); //RX,TX

//moisture sensor stuff
//#include <string.h>
int readings = 0;
byte value_list[NUM_READS][2];

//SD card stuff
#include <SPI.h>
#include "SdFat.h"
SdFat SD;
#define SD_CS_PIN SS
File myFile;
#define FILENAME "test_4inDeepDogbone.csv"

//timer stuff
unsigned long delay_start;
unsigned long curr_time;
int TIME_DELAY = 30000; //60000 is 1 minute //600000 mills is 10 minutes

//led pin stuff, if led is on then there is an error, if on then things started up normally
int ledPin = 13;

//moisture stuff
//moisture values will be taken on a seperate arduino because SDISerial library is incompatable with SoftwareSerial library
//arduinos will communicate via I2C following this tutorial http://www.instructables.com/id/I2C-between-Arduinos/
#include <Wire.h>
int moisture;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite( ledPin, HIGH );
//setup serial
  Serial.begin( 115200 ); //make sure serial monitor is set to correct baud rate
                          //115200 for RFID example
  while(!Serial); //wait until serial monitor is ready
  Serial.println();
  Serial.println(F("Initializing..."));

//setup for RFID reader
  if(setupNano(38400) == false) //function defined under RFID functions
            //configure nano to run at 38400b/s
  {
    Serial.println(F("Module failed to respond. Please check wiring to RFID shield"));
    while(1); //stop everything
  }
  digitalWrite( ledPin, LOW);
  nano.setRegion(REGION_NORTHAMERICA); //Set to North America
  nano.setReadPower(2500); //25.00 dBm. Higher values may cause USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling

//setup for SD card
  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD initialization failed!"));
    return;
  }
  Serial.println(F("SD initialization done."));
  Serial.print(F("Testing write to")); Serial.print(FILENAME);
  if( log_data("RFID value,moisture sensor, read #,freq") )                           //test sd_write 
    Serial.println(F("...write success"));
  else
    Serial.println(F("...error writing"));

//setup comm with 5TM moisture sensing arduino
  Wire.begin(); //set this arduino as master

}

void loop() {
  delay_start = millis();
  readings = 0; 
  read_tag(NUM_READS); //read tag 50 times, take the average
  delay(5000);
  get_moisture(); 
  if(sd_parse()) Serial.println(F("Data logged sucessfully!"));
  else           Serial.println(F("Error logging data"));
  curr_time = millis();
  while (curr_time < TIME_DELAY + delay_start){ curr_time = millis(); } 
}

///////////////////// RFID functions //////////////////////
//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(rfidSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  rfidSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!rfidSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(rfidSerial.available()) rfidSerial.read();
  
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
    rfidSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    rfidSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock 
}

//does NUM_READ senses and then averages it out
void read_tag( int NUM_READ ){ //modified from multiple_SEN.....
  while( readings < NUM_READ ) { 
       //list is full

  byte mySEN[4]; //Most EPCs are 12 bytes
  byte mySENlength;
  byte responseType = 0;

  while (responseType != RESPONSE_SUCCESS)//RESPONSE_IS_TAGFOUND)
  {
    mySENlength = sizeof(mySEN); //Length of EPC is modified each time .readTagEPC is called
    responseType = nano.readTagSensor402(mySEN, mySENlength, 500); //Scan for a new tag up to 500ms
    //Serial.println(F("Searching for tag"));
  }
  //Serial.print(F(" SEN["));

  //add it to list
  for( byte x = 2; x < mySENlength ; x++)
  {
    value_list[readings][x-2] = mySEN[x];
  }
  readings++;
  }
  int total = 0;
  for (byte x=0; x<readings; x++){
    total+= value_list[x][1]; //least significant bit
    total+= (value_list[x][0] * 16); //most sig bit
  }
  total /= NUM_READ;
  currSensorVal = total;
  //Serial.print( F("Average value is: ") ); Serial.print(currSensorVal);
  freq = nano.getTagFreq();
  //Serial.print( F("Last frequency is: ") ); Serial.println(freq);
  return;
}
///////////////////////SD functions////////////////////////

//info will save to CSV file with 1st column being RFID info, second being moisture sensor, 
//third being reading #, fourth being frequency
bool sd_parse(){ //writes data to sd card
  char comma = ',';
  String to_log = "";
  
  to_log += String(currSensorVal, HEX);
  to_log += comma;
  to_log += String(moisture);
  to_log += comma;
  to_log += String(delay_start/TIME_DELAY);
  to_log += comma;
  to_log += freq;
  to_log += comma;
  return log_data( to_log );
}

//write it to file
boolean log_data( String to_log ){
  myFile = SD.open( FILENAME , FILE_WRITE );
  if (myFile) {                 //if file ok, log data
    Serial.print(F("Writing to file..."));
    myFile.println( to_log );
    myFile.close();            //close the file:
    Serial.println(F("Done writing to file."));
  } else {                     //if file bad, return error
    Serial.println(F("Error opening file"));     // if the file didn't open, print an error:
    return false;
  }
  Serial.println( to_log );
  return true;
}


////////////////// 5TM moisture sensor functions ////////////////////////
//get moisture data from other arudino, save it to global variable moisture
void get_moisture(){
  Wire.requestFrom(8,4);  //request 4 byte of data from device 8
  int count = 0;
  String m_string="";
  while (Wire.available()&& count<3) { // slave may send less than requested, last byte is always curropt for some reason
    char c = Wire.read(); // receive a byte as character
    m_string+=c;
    count++;
  }
  moisture = m_string.toInt();
  
  delay(500); //delay to allow for end of Wire transmission
}
