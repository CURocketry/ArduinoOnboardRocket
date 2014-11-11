#include <SoftwareSerial.h>

#define MARKER_LAT 0xB
#define MARKER_LON 0xC
#define MARKER_ALT 0xD
#define MARKER_FLAG 0xE

#define MAX_BUF 16  // Maximum payload size 

int buf_size; //actual size of buffer
byte* buf_start = (byte*)malloc( MAX_BUF ); //allocate starting payload pointer
byte* buf_curr; //current position of buffer

//fake data
long latitude = 42443961;
long longitude = -76501881;
int altitude = 9999;
byte flags = 0xF;

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

//stream size bytes in n to ptr
byte* stream(byte marker, void* ptr, void *n, int n_size) {
  //cast ptr to a byte to allow arithmetic
  byte* p = (byte*)ptr;
  
  *p = marker; //insert marker
  p++;
  
  memcpy( p, n, n_size ); //copy size bytes of n into p
  p += n_size; //increment pointer p

  return( p );
}



