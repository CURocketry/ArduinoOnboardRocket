// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code just echos whatever is coming from the GPS unit to the
// serial monitor, handy for debugging!
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

//This code is intended for use with Arduino Leonardo and other ATmega32U4-based Arduinos
#include <math.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//make sure these match on the receiving end
#define MARKER_LAT 0xB
#define MARKER_LON 0xC
#define MARKER_ALT 0xD
#define MARKER_FLAG 0xE

//Payload variables
#define MAX_BUF 16  // Maximum payload size 
int buf_size; //actual size of buffer
byte* buf_start = (byte*)malloc( MAX_BUF ); //allocate starting payload pointer
byte* buf_curr; //current position of buffer

//fake data
long latitude = 42.4439;
long longitude = -76.5018;
int altitude = 9999;
byte flags = 0xF;

enum States {
 STOPPED, PRE_LAUNCH, POST_LAUNCH, SEND_DATA, READ_DATA, READ_GPS, ERROR 
};

States state = STOPPED;

// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 9 (Arduino's Software TX)
SoftwareSerial XBee(10,9); // RX, TX

SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
//Adafruit_GPS GPS(&Serial1);
//HardwareSerial mySerial = Serial1;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  XBee.begin(9600);
  Serial.begin(115200);
  delay(5000);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  gpsSerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c); 
  delay(200);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
          latitude = convertDegMinToDecDeg(GPS.latitude)*10000;
      longitude = convertDegMinToDecDeg(GPS.longitude)*1000;
      altitude = GPS.altitude;
      Serial.print("Location (Deg): ");
      Serial.print(GPS.latitude,4);
      Serial.print(", "); 
      Serial.println(GPS.longitude,4);
      
      Serial.print("Location (Dec): ");
      Serial.print(convertDegMinToDecDeg(GPS.latitude),5);
      Serial.print(", "); 
      Serial.println(convertDegMinToDecDeg(GPS.longitude),5);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  //set every byte in array to 0
  memset( buf_start, 0, MAX_BUF );
  
  //buf_start is pointer to first allocated space
  //buf_curr is pointer to next free space
  buf_curr = stream(MARKER_LAT, buf_start, (void*)&latitude, sizeof(latitude));
  buf_curr = stream(MARKER_LON, buf_curr, (void*)&longitude, sizeof(longitude));
  buf_curr = stream(MARKER_ALT, buf_curr, (void*)&altitude, sizeof(altitude));
  buf_curr = stream(MARKER_FLAG, buf_curr, (void*)&flags, sizeof(flags));
  
  //buffer size is the difference between the current and start pointers
  buf_size = buf_curr - buf_start;
  
  //check that buffer size did not exceed allocated size
  if( buf_size > MAX_BUF )
  {
    Serial.println("Buffer Overflow " + buf_size);
  }

  XBee.write(buf_start, buf_size);
  
  //arduino is little endian (LSB first)
  byte* bufferReader = buf_start;
  Serial.print("Size: ");
  Serial.println(buf_size);
  while (bufferReader != buf_curr) {
    Serial.print(*bufferReader, HEX);
    Serial.print(" ");
    bufferReader++;
  }
  Serial.println();
}


//stream n_size bytes in n to mem loc ptr
byte* stream(byte marker, void* ptr, void *n, int n_size) {
  //cast ptr to a byte to allow arithmetic
  byte* p = (byte*)ptr;
  
  *p = marker; //insert marker
  p++;
  
  memcpy( p, n, n_size ); //copy size bytes of n into p
  p += n_size; //increment pointer p

  return( p );
}

// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}
