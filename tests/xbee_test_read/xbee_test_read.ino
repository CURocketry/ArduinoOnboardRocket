#include <SoftwareSerial.h>

// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 9 (Arduino's Software TX)
SoftwareSerial XBee(10,9); // RX, TX

void setup()
{
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  XBee.begin(115200);
  Serial.begin(9600);
}

void loop()
{
  delay(500);
  
  if (XBee.available())
    Serial.println(XBee.read(), HEX);

}

