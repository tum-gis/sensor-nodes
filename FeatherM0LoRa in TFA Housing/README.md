# Outdoor Weather Monitoring with Feather M0 LoRa in TFA Housing

This sensor node is made to showcase a use-case of LoRaWAN technology for outdoor weather monitoring. For achieving this a Feather M0 LoRa module was used with temperature and pressure sensor. The entire setup was carefully placed in the [TFA Housing](https://www.tfa-dostmann.de/en/produkt/protective-cover-for-outdoor-transmitter/) which is an all-weather protective cover for outdoor transmitters. In this example we measure parameters such as temperature, humidity, altitude, and air pressure.

![Sensor node in TFA Housing](setup.jpg)

## Hardware

To build this sensor node we have used following hardware components:

- [Adafruit Feather M0 LoRA board](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module)
- [Grove - DHT-22 Temperature & Humidity Sensor](http://wiki.seeedstudio.com/Grove-Temperature_and_Humidity_Sensor_Pro/) 
- [Grove - Barometric Pressure Sensor](http://wiki.seeedstudio.com/Grove-Barometer_Sensor/)
- [Breadboard](https://en.wikipedia.org/wiki/Breadboard#/media/File:400_points_breadboard.jpg)
- [TFA Protective Cover](https://www.tfa-dostmann.de/en/produkt/protective-cover-for-outdoor-transmitter/)
- [6600 mAH Battery](https://www.adafruit.com/product/353)

![Inside view of Sensor node in TFA Housing](setup-insideview.jpg)

Also, as the final hardware setup with antenna couldn’t completely fit into the casing, a small hole was made at the bottom of the casing to allow the remaining portion of antenna to stay outside.

![Bottom view of Sensor node in TFA Housing](setup-bottom.jpg)

### Example: Wiring setup

First of all, the Feather M0 LoRa board was prepared by soldering the board with the provided grid of pins. Then the board is connected with the sensors using a breadboard. The sensor connections were made using the connector cables as following:

#### DHT-22 Sensor connections:
- Feather 3V to DHT22 pin 1
- Feather GND to DHT22 pin 4
- Feather pin 12 to DHT22 pin 2
- Resistor between DHT pin 1 and DHT pin 2

![Wiring with DHT-22 Sensor](feather_wiring_hero.png)

#### Grove-Barometer Sensor connections:
- Feather SCL to Barometer Sensor pin 1 (yellow)  
- Feather SDA to Barometer Sensor pin 2 (white) 
- Feather 3V to Barometer Sensor pin 3 (red)
- Feather GND to Barometer Sensor pin 4 (black)

Apart from this, Feather pin 6 should be permanently wired with Feather pin io1.

To ensure the durable connections, smaller jumper wires were used on the breadboard instead of longer connecting cables. Sensors and cables were also supported with an insulating duct tape. Final hardware setup looked as following:

![Final hardware wiring](hardware.jpg)

Once all these connection were made, the board is connected with a computer using a USB cable. Further, steps of software part needs to be followed:

## Software

To create this node, we use Arduino IDE for setting up the Feather M0 LoRa module. First, install the [Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/setup) board to your Arduino IDE and select the correct port. Then following libraries needs to be installed before compiling the code:

- [lmic.h](https://github.com/matthijskooijman/arduino-lmic/archive/master.zip) for implementing LoRaWAN on Arduino hardware.
- [hal/hal.h] bundled with lmic library.
- [Adafruit_SleepyDog.h](https://github.com/adafruit/Adafruit_SleepyDog) for controlling low power sleep mode. 
- [Wire.h](https://github.com/esp8266/Arduino/tree/master/libraries/Wire) to communicate with I2C devices.
- [BMP085.h](https://raw.githubusercontent.com/SeeedDocument/Grove-Barometer_Sensor/master/res/Barometer_Sensor.zip) for Barometer sensor.
- [DHT.h](https://github.com/Seeed-Studio/Grove_Temperature_And_Humidity_Sensor) for reading DHT-22 sensor.
- [CayenneLPP.h](https://github.com/ElectronicCats/CayenneLPP/archive/master.zip) for Cayenne Protocol.

Apart from this, SPI.h library is also used for communicating with serial peripheral interface but it is already inbuilt in Arduino IDE and is not required to be separately installed.

Now download and run the [Arduino_Sketch_TFA.ino](Arduino_Sketch_TFA/Arduino_Sketch_TFA.ino) file in the Arduino IDE. This code was created by merging the example code of both the sensors and the ttn-otaa example from the lmic library. Some required changes were made while merging the example codes. The user should change the network session key, app session key and device address in the code before compiling. These keys can be obtained from the TTN, SWM or other service providers.

```
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = {NETWORK_SESSION_KEY_HERE_IN_MSB_FORMAT};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {APPLICATION_SESSION_KEY_HERE_IN_MSB_FORMAT};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260XXXXX   ; // <-- Change this address for every node!
```
The pin mapping configured in the code should also be verified for the board that is being used. Current pin mapping is set as per the Feather M0 LoRa board. 
```
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};

/**
Set the correct pin mapping for the board that is used
**/
```

Following is the example code that can be used to measure the battery voltage of the Feather M0 LoRa board:
```
#define VBATPIN A7
measuredvbat = analogRead(VBATPIN);
measuredvbat *= 2;    // we divided by 2, so multiply back
measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
measuredvbat /= 1024; // convert to voltage

SERIALDEBUG_PRINT(" %\t");
SERIALDEBUG_PRINT("Battery Voltage: ");
SERIALDEBUG_PRINTLN(measuredvbat);
```

## Services

This node is connected using the TheThingsNetwork service. Further, a node-red work bench is used to forward this collected data from the TTN platform to the OGC Sensor Things API configured on the FROST Server. The node-red workbench that was used for forwarding the data is available at [Node Red flow for Outdoor Weather Monitoring](./Node_flow_TFA.json). To use this node-red-workbench go to the node-red platform https://iot.gis.bgu.tum.de:1885/, login with the credentials, go to the options and select Import>Clipboard. Select the downloaded .json file with the given option and click on import. Make necessary changes and deploy the flow.

Datastreams setup for this sensor node on the FROST server can be seen at:
http://iot.gis.bgu.tum.de:8081/FROST-Server-gi3/v1.0/Things(20)/Datastreams

The node-red workbench for this sensor node could be found at: https://iot.gis.bgu.tum.de:1885/#flow/f6f7a740.c6b338

The GRAFANA dash-board for visualizing the collected data is available at:
https://iot.gis.bgu.tum.de:3050/d/sMJ3jAAWz/featherm0lora-in-tfa-housing?orgId=1

## References

* [Arduino_Sketch_TFA.ino](Arduino_Sketch_TFA/Arduino_Sketch_TFA.ino)
* [Node Red flow for Outdoor Weather Monitoring](./Node_flow_TFA.json)
* [Feather M0 LoRa Arduino IDE Setup](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/setup)
