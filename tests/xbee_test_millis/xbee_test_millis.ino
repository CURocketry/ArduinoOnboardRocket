#include <SoftwareSerial.h>

#define PAYLOAD_SIZE 4

byte *payload;

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2,3); // RX, TX

void setup()
{
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  XBee.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  delay(500);
  
  //millis() returns an unsigned long
  payload = longToBytes(millis()); //convert an unsigned long to an array of bytes
  
  XBee.write(payload, PAYLOAD_SIZE); //size is always size of unsigned long, 4 bytes
  free(payload); //prevent memory leak
}

//convert a long to a byte array of length 4
byte* longToBytes(unsigned long n) {
  byte* array = new byte[4];  
  
  for (int i=0; i<4; i++) {
    array[i] = 0xFF & n;
    n = n >> 8;
  }
   return array;
 
}
