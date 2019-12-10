
 /*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 *
 *
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
// #define SERIALDEBUG
 
#ifdef SERIALDEBUG
  #define SERIALDEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define SERIALDEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define SERIALDEBUG_PRINT(...)
  #define SERIALDEBUG_PRINTLN(...)
#endif
 
 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_SleepyDog.h>
#include <DHT.h>
#include <CayenneLPP.h>
#include "BMP085.h"
#include <Wire.h>

CayenneLPP lpp(51);
 
#define DHTPIN            12        // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
 
// DHT_Unified dht(DHTPIN, DHTTYPE);
DHT dht(DHTPIN, DHTTYPE);
 
#define VBATPIN A7

float temperature2;
float pressure;
float atm;
float altitude;
BMP085 myBarometer;
 
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
 
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
 
static osjob_t sendjob;
 
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;       // seconds transmit cycle plus ...
const unsigned SLEEP_TIME = 60*9+55;  // seconds sleep time plus ...
const unsigned MEASURE_TIME = 2;      // seconds measuring time should lead to ...
                                      // 5 minute(s) total cycle time
 
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};
 
 
void onEvent (ev_t ev) {
//    Serial.print(os_getTime());
//    Serial.print(": ");
    SERIALDEBUG_PRINT(os_getTime());
    SERIALDEBUG_PRINT(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            SERIALDEBUG_PRINTLN(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            SERIALDEBUG_PRINTLN(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            SERIALDEBUG_PRINTLN(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            SERIALDEBUG_PRINTLN(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            SERIALDEBUG_PRINTLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            SERIALDEBUG_PRINTLN(F("EV_JOINED"));
            break;
        case EV_RFU1:
            SERIALDEBUG_PRINTLN(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            SERIALDEBUG_PRINTLN(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            SERIALDEBUG_PRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
            SERIALDEBUG_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              SERIALDEBUG_PRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
              SERIALDEBUG_PRINT(F("Received "));
              SERIALDEBUG_PRINT(LMIC.dataLen);
              SERIALDEBUG_PRINTLN(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
             
            SERIALDEBUG_PRINTLN("going to sleep now ... ");
            // lmic library sleeps automatically after transmission has been completed
            for(int i= 0; i < SLEEP_TIME / 16; i++) {
              Watchdog.sleep(16000); // maximum seems to be 16 seconds
              SERIALDEBUG_PRINT('.');
            }
            if (SLEEP_TIME % 16) {
              Watchdog.sleep((SLEEP_TIME % 16)*1000);
              SERIALDEBUG_PRINT('*');             
            }
            SERIALDEBUG_PRINTLN("... woke up again");
             
            break;
        case EV_LOST_TSYNC:
            SERIALDEBUG_PRINTLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            SERIALDEBUG_PRINTLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            SERIALDEBUG_PRINTLN(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            SERIALDEBUG_PRINTLN(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            SERIALDEBUG_PRINTLN(F("EV_LINK_ALIVE"));
            break;
         default:
            SERIALDEBUG_PRINTLN(F("Unknown event"));
            break;
    }
}
 
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        SERIALDEBUG_PRINTLN(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
 
        float temperature, humidity, measuredvbat;
        int16_t int16_temperature, int16_humidity, int16_vbat;
     
        // Start a measurement to update the sensor's internal temperature & humidity reading
        SERIALDEBUG_PRINTLN("Start measurement...");
        temperature = dht.readTemperature();
        // delay(2000);
        Watchdog.sleep(2000);
        // Now read the recently measured temperature (2 secs ago) as Celsius (the default)
        temperature = dht.readTemperature();
        // Read the recently measured humidity (2 secs ago)
        humidity = dht.readHumidity();
        SERIALDEBUG_PRINTLN("... finished!");
     
        // Check if any reads failed and exit early (to try again).
        if (isnan(humidity) || isnan(temperature)) {
            SERIALDEBUG_PRINTLN("Failed to read from DHT sensor!");
            for (int i=0; i<5; i++) {
              digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on by making the voltage HIGH                   
              delay(150);
              digitalWrite(LED_BUILTIN, LOW);    // turn the LED on by making the voltage HIGH                   
              delay(150);
            }
            // ok, then wait for another period and try it again
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        } else {
            SERIALDEBUG_PRINT("Humidity: ");
            SERIALDEBUG_PRINT(humidity);
            SERIALDEBUG_PRINT(" %\t");
            SERIALDEBUG_PRINT("Temperature: ");
            SERIALDEBUG_PRINT(temperature);
            SERIALDEBUG_PRINT(" *C ");
 
            measuredvbat = analogRead(VBATPIN);
            measuredvbat *= 2;    // we divided by 2, so multiply back
            measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
            measuredvbat /= 1024; // convert to voltage

            SERIALDEBUG_PRINT(" %\t");
            SERIALDEBUG_PRINT("Battery Voltage: ");
            SERIALDEBUG_PRINTLN(measuredvbat);

            temperature2 = myBarometer.bmp085GetTemperature(myBarometer.bmp085ReadUT()); //Get the temperature, bmp085ReadUT MUST be called first
            pressure = myBarometer.bmp085GetPressure(myBarometer.bmp085ReadUP());//Get the temperature
   
           /*
            To specify a more accurate altitude, enter the correct mean sea level
            pressure level.  For example, if the current pressure level is 1019.00 hPa
            enter 101900 since we include two decimal places in the integer value。
           */
           altitude = myBarometer.calcAltitude(pressure); 
           
           atm = pressure / 101325;
        
           lpp.reset();
           lpp.addTemperature(1, temperature);
           lpp.addRelativeHumidity(2, humidity);
           lpp.addAnalogInput(3, measuredvbat);
           lpp.addTemperature(4, temperature2);
           lpp.addBarometricPressure(5, pressure/100);
           lpp.addAnalogInput(6, atm);
           lpp.addAnalogInput(7, altitude);
            
//            LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
 
            // send the 6 bytes payload to LoRaWAN port 7
            LMIC_setTxData2(7, lpp.getBuffer(), lpp.getSize(), 0);
            SERIALDEBUG_PRINTLN(F("Packet queued")); 
            digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on by making the voltage HIGH
        }
         
        // LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        // Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
 
void setup() {
    delay(5000);
 
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
 
#ifdef SERIALDEBUG
    Serial.begin(9600);
    // while (!Serial);
#endif
 
    dht.begin();
    myBarometer.init(); 
    SERIALDEBUG_PRINTLN(F("Starting"));
 
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
 
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
 
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
 
    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif
 
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
 
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
 
    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
 
    // Start job
    do_send(&sendjob);
}
 
void loop() {
    os_runloop_once();
}
