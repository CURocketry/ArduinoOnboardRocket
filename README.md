LaunchVehicleController
========================
Interface between launch vehicle and ground station - monitors launch vehicle status, executes instructions, and communicates with the ground station.

##Summary
The LaunchVehicleController is an Arduino-backed module with current support for reading GPS data, communicating with a Ground Station Controller, and monitoring launch vehicle status.

##Design Goals
Several high level design goals were considered during the development process. These should be refined and adhered to during future development.
* Monitor flight status from on-board sensors.
* Transmitting: Coordinate status transmission from the launch vehicle to the ground station.
* Receiving: Coordinate directive reception and processing on the launch vehicle from the ground station.
* Extensibility: Easily support new input sensors and output controls.
* Stability: Gracefully degrade in the event of sensor malfunction, and separate processing logic from transmission to be able to report on system failure.

##Hardware Requirements
The software has been tested with the following hardware. Any changes may introduce bugs or require software changes.
* Arduino Micro / Leonardo
* XBee S3B 900MHz Wireless Module
* Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3

##Setup Instructions
Construct the module considering pin outputs defined in `onboard_controller.ino`. By default:
* XBee DOUT (TX) is connected to pin 0 (Arduino's Hardware RX).
* XBee DIN (RX) is connected to pin 1 (Arduino's Hardware TX). 
* GPS DOUT (TX) is connected to pin 8 (Arduino's Software RX).
* GPS DIN (RX) is connected to pin 7 (Arduino's Software TX). 

To debug the set up and ensure that individual components are functional, it may be desirable to run the necessary programs in the `tests` directory.

##Project Structure
```
├── onboard_controller
│   ├── onboard_controller.ino
│   ├── payload_def.h
└── tests
│   └── ...
```
* [onboard_controller.ino](/onboard_controller/onboard_controller.ino) contains the main processing logic of the module. 
* [payload_def.h](/onboard_controller/payload_def.h) contains the implementation of the communication [protocol](#communication-protocol).
* [tests](/tests) contains iterative tests that may be deployed to ensure individual components are functional.

##Parameters
In `onboard_controller.ino`, uncommenting the following macros may assist in debugging:
* `readDebug` - Prints all Serial1 reads to console.
* `writeDebug` - Prints all Serial1 writes to console.
* `fakeGPS` - Fakes the presence of a GPS (sets payload.latitude, payload.longitude, payload.altitude, and payload.flags.gps_fix).
* `rawGPSDebug` - Prints the raw GPS data output to console.

Additional configuration parameters are as follows:
* `LAUNCH_TX_PD` - Period of data transmission (ms) during launch. Active on receipt of the `DIR_BEGIN_LAUNCH` directive.
* `STANDBY_TX_PD` - Period of data transmission (ms) during standby. Active when the program first starts.
* `PIN_TEST` - An output test pin that toggles state on receipt of `DIR_TEST` directive. May be useful for testing.
* `PIN_PAYLOAD_ABORT` - An output pin that toggles the cancelling of a payload abort mechanism. Held HIGH on receipt of `DIR_PAYLOAD_ABORT` directive and held LOW on receipt of `DIR_PAYLOAD_ABORT_CANCEL` directive.

##Communication Protocol
For detailed information on the communication protocol, refer to [protocol.md](/protocol.md). 

##Notes
* The Arduino Micro/Leonardo share similar pin outputs, and were chosen for their separate USB serial (Serial) and Rx/Tx pin serial (Serial1). This enables easier debugging without requiring task scheduling, since debug information can be printed independently of message transmission.
* The Arduino Micro/Leonardo have the same change interrupt pins, which are different from those of other Arduinos. Take note when selecting RX pins to use with SoftwareSerial. From the Arduino documentation, the following pins on the Arduino Micro/Leonardo support change interrupts: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
* The ATMega chips are little-endian and consequently, transmitted raw data follows a little-endian format within each chunk.