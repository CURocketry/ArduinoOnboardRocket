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
long latitude = 0;//42.4439;
long longitude = 0;//-76.5018;
int altitude = 0;//9999
byte flags = 0xF;

enum States {
  STOPPED, PRE_LAUNCH, POST_LAUNCH, SEND_DATA, READ_DATA, READ_GPS, ERROR 
};

States state = STOPPED;

// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 9 (Arduino's Software TX)
SoftwareSerial XBee(10,9); // RX, TX

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

void setup()
{
  //Make sure XBee baud matches hardware config
  XBee.begin(9600);
  Serial.begin(115200);
  //delay(5000);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  //RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //"minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1/5 Hz

  // Request updates on antenna status
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

}

uint32_t timer = millis();

void loop()
{
  //request GPS data
  GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, reset
  if (timer > millis())  timer = millis();

  if (millis() - timer > 400) {
    timer = millis(); // reset the timer

    if (GPS.fix) {
      latitude = convertDegMinToDecDeg(GPS.latitude)*10000;
      longitude = convertDegMinToDecDeg(GPS.longitude)*10000;
      altitude = GPS.altitude;

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
  }
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






