#include <math.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "payload_def.h"

/*--debug macros--*/
//#define readDebug //print Serial1 reads to console
//#define writeDebug //print Serial1 writes to console
#define fakeGPS //fake the presence of a gps
//#define rawGPSDebug //print raw gps data to console

#define PIN_TEST 13
#define PIN_PAYLOAD_ABORT 12

//Payload variables
#define MAX_BUF 16  // Maximum payload size 
int buf_size; //actual size of buffer
byte* buf_start = (byte*)malloc( MAX_BUF ); //allocate starting payload pointer
byte* buf_curr; //current position of buffer

Payload payload;
Payload payload_start;
Payload* ptr_payload = &payload;

enum States {
  STOPPED, PRE_LAUNCH, POST_LAUNCH, ERROR 
};

States state = STOPPED;

// XBee's DOUT (TX) is connected to pin 0 (Arduino's Hardware RX)
// XBee's DIN (RX) is connected to pin 1 (Arduino's Hardware TX)

SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);

void setup()
{
  pinMode(PIN_TEST, OUTPUT);
  pinMode(PIN_PAYLOAD_ABORT, OUTPUT);
  
  //Make sure Serial baud matches hardware config
  Serial1.begin(57600); //XBee baud rate
  Serial.begin(115200);

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
  gpsSerial.println(PMTK_Q_RELEASE);

}

uint32_t timer = millis();

void loop()
{
  #ifdef rawGPSDebug
    while (gpsSerial.available()) {
      char c = GPS.read();
      if (c) {
        Serial.write(c);
      } 
    }
  #endif
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
 
  //handle XBee received data
  if (Serial1.available()) {
    byte receivedPayload = Serial1.read();
    switch (receivedPayload) {
      case DIR_TEST:
        if ( digitalRead(PIN_TEST) == LOW ) {
          digitalWrite(PIN_TEST,HIGH);
          payload.flags.test = 1;
        }
        else {
          digitalWrite(PIN_TEST,LOW);
          payload.flags.test = 0;
        }
        break;
      case DIR_PAYLOAD_ABORT:
        digitalWrite(PIN_PAYLOAD_ABORT, HIGH);
        payload.flags.payload_abort = 1;
        break;
      case DIR_PAYLOAD_ABORT_CANCEL:
        digitalWrite(PIN_PAYLOAD_ABORT, LOW);
        payload.flags.payload_abort = 0;
        break;
      case DIR_BEGIN_LAUNCH:
        
        break;
      default:
        digitalWrite(PIN_TEST,LOW);
        break;
    }
    #ifdef readDebug
      Serial.print("read size: ");
      Serial.println(Serial.available());
      Serial.println(receivedPayload, HEX);
    #endif
  }
  
  // if millis() or timer wraps around, reset
  if (timer > millis())  timer = millis();
  if (millis() - timer > 250) {
    timer = millis(); // reset the timer
    
    #ifdef fakeGPS //fake gps data
      payload.latitude = 42.4439*10000;
      payload.longitude = 76.5018*10000;
      payload.altitude = 9999;
    #else    
      if (GPS.fix) {
        setGPSPayload(payload);
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
      
      //arduino is little endian (LSB first)
      Serial1.write(buf_start, buf_size);
      
      #ifdef writeDebug
        //check that buffer size did not exceed allocated size
        if( buf_size > MAX_BUF )
        {
          Serial.println("Buffer Overflow " + buf_size);
        }
        //read the contents of the buffer
        byte* bufferReader = buf_start;
        Serial.print("write size: ");
        Serial.println(buf_size);
        while (bufferReader != buf_curr) {
          Serial.print(*bufferReader, HEX);
          Serial.print(" ");
          bufferReader++;
        }
        Serial.println();
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

Payload setGpsPayload(Payload payload) {
  payload.latitude = convertDegMinToDecDeg(GPS.latitude)*10000;
  payload.longitude = convertDegMinToDecDeg(GPS.longitude)*10000;
  payload.altitude = GPS.altitude;
  return payload;
}
