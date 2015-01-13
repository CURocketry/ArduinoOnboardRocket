#include <math.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

/*--debug macros--*/
#define readDebug //print xbee reads to console
//#define writeDebug //print xbee writes to console
#define fakeGPS //fake the presence of a gps

//make sure these match on the receiving end
#define MARKER_LAT 0xFB
#define MARKER_LON 0xFC
#define MARKER_ALT 0xFD
#define MARKER_FLAG 0xFE
#define FLAG_GPS_FIX 0b00000001
#define FLAG_PAYLOAD 0b00000010

//make sure these match on the sending end
#define FLAG_PAYLOAD 0xAB

#define PIN_PAYLOAD 13

//Payload variables
#define MAX_BUF 16  // Maximum payload size 
int buf_size; //actual size of buffer
byte* buf_start = (byte*)malloc( MAX_BUF ); //allocate starting payload pointer
byte* buf_curr; //current position of buffer

//fake data
struct Payload {
  long latitude; //42.4439;
  long longitude;//-76.5018;
  int altitude;//9999
  byte flags;//0xF;
} payload;

Payload* ptr_payload = &payload;

enum States {
  STOPPED, PRE_LAUNCH, POST_LAUNCH, SEND_DATA, READ_DATA, READ_GPS, ERROR 
};

States state = STOPPED;

// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 9 (Arduino's Software TX)
SoftwareSerial XBee(10,9); // RX, TX

SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);

void setup()
{
  pinMode(PIN_PAYLOAD, OUTPUT);
  
  //Make sure XBee baud matches hardware config
  XBee.begin(115200);
  Serial.begin(115200);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  //RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //"minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1/5 Hz

  // Request updates on antenna status
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  gpsSerial.println(PMTK_Q_RELEASE);

}

uint32_t timer = millis();

void loop()
{
  gpsSerial.listen(); //switch to GPS serial
  while (gpsSerial.available()) {
    char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c))
    Serial.write(c); 
  }
  XBee.listen();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
 
  // if millis() or timer wraps around, reset
  if (timer > millis())  timer = millis();
  if (millis() - timer > 400) {
    timer = millis(); // reset the timer

    if (XBee.available()) {
      byte receivedPayload = XBee.read();
      switch (receivedPayload) {
        case FLAG_PAYLOAD:
          digitalWrite(PIN_PAYLOAD,HIGH);
          break;
        default:
          digitalWrite(PIN_PAYLOAD,LOW);
          break;
      }
      #ifdef readDebug
        Serial.print("size: ");
        Serial.println(XBee.available());
        Serial.println(receivedPayload, HEX);
      #endif
    }
    
    #ifdef fakeGPS //fake gps data
      payload.latitude = 42.4439*10000;
      payload.longitude = 76.5018*10000;
      payload.altitude = 9999;
      state = SEND_DATA;
    #else    
      if (GPS.fix) {
        payload.latitude = convertDegMinToDecDeg(GPS.latitude)*10000;
        payload.longitude = convertDegMinToDecDeg(GPS.longitude)*10000;
        payload.altitude = GPS.altitude;
        state = SEND_DATA;
      }
      else {
        payload.latitude = 0;
        payload.longitude = 0;
        payload.altitude = 0;
        
        #ifdef debug
          Serial.println("No fix");
        #endif
    #endif
      
      //set every byte in array to 0
      memset( buf_start, 0, MAX_BUF );

      //buf_start is pointer to first allocated space
      //buf_curr is pointer to next free space
      if (payload.latitude != 0 && payload.longitude != 0 && payload.altitude != 0) {
        buf_curr = stream(MARKER_LAT, buf_start, (void*)&(ptr_payload->latitude), sizeof(payload.latitude));
        buf_curr = stream(MARKER_LON, buf_curr, (void*)&(ptr_payload->longitude), sizeof(payload.longitude));
        buf_curr = stream(MARKER_ALT, buf_curr, (void*)&(ptr_payload->altitude), sizeof(payload.altitude));
      }
      
      buf_curr = stream(MARKER_FLAG, buf_curr, (void*)&(ptr_payload->flags), sizeof(payload.flags));

      //buffer size is the difference between the current and start pointers
      buf_size = buf_curr - buf_start;

      XBee.write(buf_start, buf_size);
      //arduino is little endian (LSB first)
      
      #ifdef writeDebug
        //check that buffer size did not exceed allocated size
        if( buf_size > MAX_BUF )
        {
          Serial.println("Buffer Overflow " + buf_size);
        }
        //read the contents of the buffer
        byte* bufferReader = buf_start;
        Serial.print("Size: ");
        Serial.println(buf_size);
        while (bufferReader != buf_curr) {
          Serial.print(*bufferReader, HEX);
          Serial.print(" ");
          bufferReader++;
        }
        Serial.println();
        
          if (XBee.overflow()) {
           Serial.println("XBee serial buffer overflow!");
          } 
      #endif
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
