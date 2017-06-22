#include <Wire.h>

#include <SDISerial.h>
#include <string.h>
#define DATA_PIN 6
SDISerial connection(DATA_PIN);
char output_buffer[125]; // just for uart prints
char tmp_buffer[4];
char* resp; // Pointer to response char
char noMessageError[] = "No Response Recieved!!";
String moisture = "";

void setup() {
  connection.begin();
  Serial.begin(9600);
  char* sensor_info = connection.sdi_query("?I!",1000); // get sensor info
  //print to uart
  sprintf(output_buffer,"Sensor Info: %s",sensor_info?sensor_info:"No Response");
  Serial.println(output_buffer);

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  
}

void loop() {
  moisture = get_moisture();
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Serial.println(moisture);
  char message[4];
  moisture.toCharArray(message,4);
  Wire.write(message); // respond with message of 6 bytes
  Serial.println("message sent");
  // as expected by master
}
String get_moisture(){
    connection.begin();
    Serial.println("Begin Command: ?M!");   
    //send measurement query (M) to the first device on our bus
    resp = connection.sdi_query("?M!",1000);//1 second timeout
    //this really just returns a message that tells you the maximum wait before the measurement is ready
    
    sprintf(output_buffer,"RECV: %s",resp?resp:noMessageError);
    Serial.println(output_buffer);
    delay(500);//sleep for 1 seconds before the next command
    
    //print to uart
    Serial.println("Begin Command: ?D0!");
    resp = connection.sdi_query("?D0!",1000);//1 second timeout
    
    sprintf(output_buffer,"RECV: %s",resp?resp:noMessageError);
    Serial.println(output_buffer);
    return parseMoisture();

}
String parseMoisture(){
  if (!resp) return "666";
  
  String total = "";
  total += String(resp[2])+String(resp[3])+String(resp[4])+" ";
  Serial.println(total);
  return total;
}
