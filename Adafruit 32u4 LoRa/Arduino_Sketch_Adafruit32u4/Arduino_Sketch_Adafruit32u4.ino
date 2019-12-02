/*******************************************************************************
 * Arduino Sketch for a LoRaWAN sensor node that is registered with
 * 'The Things Network' (TTN) www.thethingsnetwork.org
 *
 * Author:  Thomas H. Kolbe, thomas.kolbe@tum.de
 * Version: 1.0
 * Last update: 2018-05-21
 *
 * The sensor node is based on the Adafruit Feather 32u4 LoRa microcontroller
 * board https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/
 * The sensor node uses a DHT22 sensor measuring air temperature and humidity.
 * Also the voltage of an attached LiPo battery is monitored and sent as
 * an observation. All three values are encoded as 2 byte integer values each.
 * Hence, the total message payload is 6 bytes. Before the values are converted
 * to integers they are multiplied by 100 to preserve 2 digits after the decimal
 * point. Thus, the received values must be divided by 100 to obtain the measured
 * values. The payload is sent every 300s to LoRaWAN port 7. The following
 * Javascript function can be used as a payload decoding function in TTN:
 *
 * function Decoder(bytes, port) {
 *   // Decode an uplink message from a buffer
 *   // (array) of bytes to an object of fields.
 *   if (port==7) {
 *     var decoded = {
 *       "temperature": (bytes[0] << 8 | bytes[1]) / 100.0,
 *       "humidity": (bytes[2] << 8 | bytes[3]) / 100.0,
 *       "vbattery": (bytes[4] << 8 | bytes[5]) / 100.0
 *     };
 *   } else {
 *     var decoded = null;
 *   }
 *   return decoded;
 * }
 *
 * In between two data transmissions the microcontroller board can go
 * into sleep mode to reduce energy consumption for extended operation
 * time when running on battery. Usage of the sleep mode must be
 * explicitly configured below.
 *
 * Important hint: everytime the sensor node is reset or being started again,
 * make sure to reset the frame counter of the registered sensor in the
 * TTN console at https://console.thethingsnetwork.org. The reason is that
 * in LoRaWAN all packets that are transmitted have a frame counter, which
 * is incremented after each data frame being sent. This way a LoRaWAN application
 * can avoid receiving and using the same packet again (replay attack). When
 * TTN receives a data packet, it checks if the frame number is higher than
 * the last one received before. If not, the received packet is considered
 * to be old or a replay attack and is discarded. When the sensor node is
 * reset or being started again, its frame counter is also reset to 0, hence,
 * the TTN application assumes that all new packages are old, because their
 * frame counter is lower than the last frame received (before the reset).
 *
 * Note, that the DHT22 data pin must be connected to Digital Pin 5 of the
 * microcontroller board. A resistor of 4.7k - 10k Ohm must be connected to
 * the data pin and VCC (+3.3V). Digital Pin 6 must be connected to IO1 of the
 * LoRa transceiver module using a simple wire.
 *
 * The code is based on the Open Source library LMIC implementing the LoRaWAN
 * protocol stack on top of a given LoRa transceiver module (here: RFM95 from
 * HopeRF, which uses the Semtech SX1276 LoRa chip). The library is originally
 * being developed by IBM and has been ported to the Arduino platform. See
 * notes below from the original developers.
 *
 *******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
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
 
// If the following line is uncommented, messages are being printed out to the
// serial connection for debugging purposes. When using the Arduino Integrated
// Development Environment (Arduino IDE), these messages are displayed in the
// Serial Monitor selecting the proper port and a baudrate of 115200.
 
#define SERIALDEBUG
 
#ifdef SERIALDEBUG
  #define SERIALDEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define SERIALDEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define SERIALDEBUG_PRINT(...)
  #define SERIALDEBUG_PRINTLN(...)
#endif
 
// If the following line is uncommented, the sensor node goes into sleep mode
// in between two data transmissions. Also the 2secs time between the
// initialization of the DHT22 sensor and the reading of the observations
// is spent in sleep mode.
// Note, that on the Adafruit Feather 32u4 LoRa board the Serial connection
// gets lost as soon as the board goes into sleep mode, and it will not be
// established again. Thus, the definition of SERIALDEBUG should be commented
// out above when using sleep mode.
 
#define SLEEPMODE
 
#ifdef SLEEPMODE
#include <Adafruit_SleepyDog.h>
#endif
 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
 
#include <DHT.h>
 
#define DHTPIN            5       // Arduino Digital Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22   // DHT 22 (AM2302)
 
DHT dht(DHTPIN, DHTTYPE);         // create the sensor object
 
#define VBATPIN A9                // battery voltage is measured from Analog Input A9
 
// The following three constants (NwkSKey, AppSKey, DevAddr) must be changed
// for every new sensor node. We are using the LoRaWAN ABP mode (activation by
// personalisation) which means that each sensor node must be manually registered
// in the TTN console at https://console.thethingsnetwork.org before it can be
// started. In the TTN console create a new device and choose ABP mode in the
// settings of the newly created device. Then, let TTN generate the NwkSKey and
// and the AppSKey and copy them (together with the device address) from the webpage
// and paste them below.
 
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
 
// The following array of bytes is a placeholder to contain the message payload
// which is transmitted to the LoRaWAN gateway. We are currently only using 6 bytes.
// Please make sure to extend the size of the array, if more sensors should be
// attached to the sensor node and the message payload becomes larger than 10 bytes.
static uint8_t mydata[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xA}; 
 
static osjob_t sendjob;
 
// Schedule transmission every TX_INTERVAL seconds (might become longer due to duty
// cycle limitations). The total interval time is 2secs for the measurement
// plus 3secs for the LoRaWAN packet transmission plus TX_INTERVAL_AFTER_SLEEP seconds
// plus SLEEP_TIME seconds (microcontroller in sleep mode)
const unsigned TX_INTERVAL = 300;       // overall cycle time (send one set of observations every 5mins)
const unsigned TX_WAIT_AFTER_SLEEP = 1; // seconds to wait after return from sleep mode before the next transmit is scheduled
const unsigned TX_TIME = 3;             // rough estimate of transmission time of a single packet
const unsigned MEASURE_TIME = 2;        // seconds measuring time
const unsigned SLEEP_TIME = TX_INTERVAL - TX_WAIT_AFTER_SLEEP - TX_TIME - MEASURE_TIME;
const unsigned WAIT_TIME = TX_INTERVAL - TX_TIME - MEASURE_TIME;
 
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
};
 
void onEvent (ev_t ev) {
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
#ifdef SERIALDEBUG
              SERIALDEBUG_PRINT(F("Received "));
              SERIALDEBUG_PRINT(LMIC.dataLen);
              SERIALDEBUG_PRINT(F(" bytes of payload: 0x"));
              for (int i=0; i<LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                  SERIALDEBUG_PRINT(F("0"));
                }
                SERIALDEBUG_PRINT(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
              SERIALDEBUG_PRINTLN();
#endif
              // add your code to handle a received downlink data packet here
            }
 
#ifdef SLEEPMODE           
            // Schedule next transmission in 1 second after the board returns from sleep mode
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_WAIT_AFTER_SLEEP), do_send);
             
            SERIALDEBUG_PRINTLN("going to sleep now ... ");
            // lmic library sleeps automatically after transmission has been completed
            for(int i= 0; i < SLEEP_TIME / 8; i++) {
              Watchdog.sleep(8000); // maximum seems to be 8 seconds
              SERIALDEBUG_PRINT('.');
            }
            if (SLEEP_TIME % 8) {
              Watchdog.sleep((SLEEP_TIME % 8)*1000);
              SERIALDEBUG_PRINT('*');             
            }
            SERIALDEBUG_PRINTLN("... woke up again");
#else
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(WAIT_TIME), do_send);
#endif          
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
     
        // Start a measurement to update the sensor's internal temperature & humidity reading.
        // Note, that when fetching measurements from a DHT22 sensor, the reported
        // values belong to the measurement BEFORE the current measurement.
        // Therefore, in order to get current observations, we first perform a new measurement
        // and wait 2 secs (which is the minimum time between two sensor observations for
        // the DHT22) and then directly retrieve the observations again.
        temperature = dht.readTemperature();
#ifdef SLEEPMODE
        Watchdog.sleep(2000);
#else
        delay(2000);
#endif       
        // Now read the recently measured temperature (2 secs ago) as Celsius (the default)
        temperature = dht.readTemperature();
        // Read the recently measured humidity (2 secs ago)
        humidity = dht.readHumidity();
     
        // Check if any reads failed and exit early (to try again).
        if (isnan(humidity) || isnan(temperature)) {
            SERIALDEBUG_PRINTLN("Failed to read from DHT sensor!");
            // blink the LED five times to indicate that the sensor values could not be read
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
            SERIALDEBUG_PRINT(" °C ");
 
            int16_temperature = 100*temperature;
            int16_humidity = 100*humidity;
            mydata[0] = (byte) (int16_temperature >> 8);
            mydata[1] = (byte) (int16_temperature & 0x00FF);
            mydata[2] = (byte) (int16_humidity >> 8);
            mydata[3] = (byte) (int16_humidity & 0x00FF);
 
            measuredvbat = analogRead(VBATPIN);
            measuredvbat *= 2;    // we divided by 2, so multiply back
            measuredvbat *= 3.3;   // Multiply by 3.3V, our reference voltage
            measuredvbat /= 1023;  // convert to voltage
            int16_vbat = round(measuredvbat * 100);
            mydata[4] = (byte) (int16_vbat >> 8);
            mydata[5] = (byte) (int16_vbat & 0x00FF);
            SERIALDEBUG_PRINT(" \t");
            SERIALDEBUG_PRINT("Battery Voltage: ");
            SERIALDEBUG_PRINT(measuredvbat);
            SERIALDEBUG_PRINTLN(" V");
            
            // Send the 6 bytes payload to LoRaWAN port 7 and do not request an acknowledgement.
            // The following call does not directly sends the data, but puts a "send job"
            // in the job queue. This job eventually is performed in the call "os_runloop_once();"
            // issued repeatedly in the "loop()" method below. After the transmission is
            // complete, the EV_TXCOMPLETE event is signaled, which is handled in the
            // event handler method "onEvent (ev_t ev)" above. In the EV_TXCOMPLETE branch
            // then a new call to the "do_send(osjob_t* j)" method is being prepared for
            // delayed execution with a waiting time of TX_INTERVAL seconds.
            LMIC_setTxData2(7, mydata, 6, 0);
            SERIALDEBUG_PRINTLN(F("Packet queued")); 
            digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on by making the voltage HIGH
             
            // Next TX is scheduled after TX_COMPLETE event.
        }
    }
}
 
void setup() {
    delay(5000);                    // give enough time to open serial monitor (if needed)
 
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
 
#ifdef SERIALDEBUG
    Serial.begin(115200);
    // while (!Serial);
#endif
 
    dht.begin();                    // initialize DHT22 sensor
     
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
 
    // Start job. This will initiate the repetitive sending of data packets,
    // because after each data transmission, a delayed call to "do_send()"
    // is being scheduled again.
    do_send(&sendjob);
}
 
void loop() {
    os_runloop_once();
}
