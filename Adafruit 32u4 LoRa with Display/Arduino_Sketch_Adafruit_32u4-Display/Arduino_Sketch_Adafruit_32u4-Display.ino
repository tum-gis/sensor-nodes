/*******************************************************************************
 * Arduino Sketch for a LoRaWAN sensor node that is registered with
 * 'The Things Network' (TTN) www.thethingsnetwork.org
 *
 * Author:  Thomas H. Kolbe, thomas.kolbe@tum.de
 * Version: 1.0.0
 * Last update: 2018-12-09
 *
 * The sensor node is based on the Adafruit Feather LoRa microcontroller board
 * with either the AVR ATmega32u4 or the ATSAMD21G18 ARM Cortex M0 microcontroller.
 * See https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/
 * or https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/
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
 * Note, that the DHT22 data pin must be connected to Digital Pin 6 of the
 * microcontroller board (for the Feather 32u4) or Digital Pin 12 (for
 * the Feather M0). A resistor of 4.7k - 10k Ohm must be connected to
 * the data pin and VCC (+3.3V).
 *
 * Digital Pin 5 (for the Feather 32u4) must be connected to DIO1 of the
 * LoRa transceiver module using a simple wire.
 *
 * For this node we also attach an Adafruit Feather Wing with a four digit
 * 14-segments LED display. The display controller is using I2C and the
 * I2C pins SDA and SCL are directly connected to the Adafruit Feather
 * via the Wing connectors. The wing is using the default I2C address
 * (0x70). Any LoRaWAN downlink message sent to this node is shown on
 * the display (only the first 4 characters). We treat each byte of the
 * received payload as a character in ASCII code. Besides numbers and
 * letters in upper and lower case also some special characters are
 * supported. For further details on the Feather Display Wing see here:
 * https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing
 *
 * In order to notify persons standing nearby that a new text was received
 * we let the node beep a couple of times. Therefore, Digital Pin 12 (for
 * the Feather 32u4) should be connected to the '+' port of a 3.3V buzzer
 * module. The '-' port of the buzzer must be connected to GND.
 * If a payload containing just a single space (character code 0x20) is
 * received, the display will be blanked without emitting beeps.
 *
 * Note that if the LED display shows some text this will draw a
 * significant amount of power. This will certainly reduce the operational
 * duration when running on battery.
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
 * This uses OTAA (Over-the-air activation), where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
 
// If the following line is uncommented, messages are being printed out to the
// serial connection for debugging purposes. When using the Arduino Integrated
// Development Environment (Arduino IDE), these messages are displayed in the
// Serial Monitor selecting the proper port and a baudrate of 115200.
 
// #define SERIALDEBUG
 
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
// out above when using sleep mode with this board.
 
#define SLEEPMODE
 
#ifdef SLEEPMODE
  #include <Adafruit_SleepyDog.h>
#endif
 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
 
#include <util/atomic.h>
#include <avr/power.h>
 
#include <DHT.h>
 
#ifdef __AVR_ATmega32U4__
  #define DHTPIN            6     // Arduino Digital Pin which is connected to the DHT sensor for Feather 32u4.
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
  #define DHTPIN            12    // Arduino Digital Pin which is connected to the DHT sensor for Feather M0.
#endif
#define DHTTYPE          DHT22    // DHT 22 (AM2302)
 
DHT dht(DHTPIN, DHTTYPE);         // create the sensor object
 
 
#ifdef __AVR_ATmega32U4__
   #define VBATPIN A9             // battery voltage is measured from Analog Input A9 for Feather 32u4
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
   #define VBATPIN A7             // battery voltage is measured from Analog Input A7 for Feather M0
#endif
 
#ifdef __AVR_ATmega32U4__
  extern volatile unsigned long timer0_overflow_count;
#endif
 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
 
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
 
#define BUZZERPIN 12              // Arduino Digital Pin which is connected to the buzzer module
 
 
// The following three constants (AppEUI, DevEUI, AppKey) must be changed
// for every new sensor node. We are using the LoRaWAN OTAA mode (over the
// air activation). Each sensor node must be manually registered in the
// TTN console at https://console.thethingsnetwork.org before it can be
// started. In the TTN console create a new device and enter the DevEUI
// number that was shipped with the Adafruit Feather LoRa board. Note that
// the shipped number only consists of 6 bytes while LoRaWAN requires
// an 8 bytes DevEUI. We simply add 0x00 0x00 in the middle of the 6 bytes
// provided. If you have lost the provided DevEUI you can also let the
// TTN console create a new one. After the registration of the device the
// three values can be copied from the TTN console. A detailed explanation
// of these steps is given in
// https://learn.adafruit.com/the-things-network-for-feather?view=all
 
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
 
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xFF, 0xGG, 0xHH, 0xII, 0xJJ, 0xB6, 0x76, 0x98 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
 
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xCC, 0xDD, 0xEE, 0xAA, 0xBB, 0xDD, 0xFF, 0xDD, 0x55, 0xDD, 0x8A, 0xDC, 0x28, 0x40, 0xBB, 0xE7 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
 
 
// The following array of bytes is a placeholder to contain the message payload
// which is transmitted to the LoRaWAN gateway. We are currently only using 6 bytes.
// Please make sure to extend the size of the array, if more sensors should be
// attached to the sensor node and the message payload becomes larger than 10 bytes.
static uint8_t mydata[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xA}; 
 
static osjob_t sendjob;
 
// Schedule transmission every TX_INTERVAL seconds (might become longer due to duty
// cycle limitations). The total interval time is 2secs for the measurement
// plus 3secs for the LoRaWAN packet transmission plus SLEEP_TIME seconds
// plus SLEEP_TIME seconds (microcontroller in sleep mode)
const unsigned int TX_INTERVAL = 300;       // overall cycle time (send one set of observations every 5mins)
const unsigned int TX_TIME = 22;            // rough estimate of transmission time of a single packet
const unsigned int MEASURE_TIME = 2;        // seconds measuring time
const unsigned int SLEEP_TIME = TX_INTERVAL - TX_TIME - MEASURE_TIME;
const unsigned int WAIT_TIME = TX_INTERVAL - TX_TIME - MEASURE_TIME;
 
// Pin mapping of the LoRa transceiver. Please make sure that DIO1 is connected
// to Arduino Digital Pin 6 using an external wire. DIO2 is left unconnected
// (it is only required, if FSK modulation instead of LoRa would be used).
#ifdef __AVR_ATmega32U4__
  const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 5, LMIC_UNUSED_PIN},     // in the Feather 32u4 DIO0 is connected to Arduino Digital Pin 7
  };
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
  const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},     // in the Feather M0 DIO0 is connected to Arduino Digital Pin 3
  };
#endif
 
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
 
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
//            LMIC_setLinkCheckMode(0);
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
              alpha4.clear();
              for (int i=0; i<LMIC.dataLen && i<4; i++) {
                  alpha4.writeDigitAscii(i, LMIC.frame[LMIC.dataBeg + i]);
              }
              alpha4.writeDisplay();
              if (!(LMIC.frame[LMIC.dataBeg]==' ' && LMIC.dataLen==1))
                  messagebeep();
            }
 
#ifdef SLEEPMODE           
            // Schedule next transmission in 1ms second after the board returns from sleep mode
            os_setTimedCallback(&sendjob, os_getTime()+ms2osticks(1), do_send);
             
            SERIALDEBUG_PRINTLN("going to sleep now ... ");
            // lmic library sleeps automatically after transmission has been completed
 
            doSleep((uint32_t)SLEEP_TIME*1000);
/*           
            int sleepcycles = (int)SLEEP_TIME / 8;
            int restsleep = (int)SLEEP_TIME % 8;
            for(int i=0; i < sleepcycles; i++) {
              Watchdog.sleep(8000); // maximum seems to be 8 seconds
              SERIALDEBUG_PRINT('.');
            }
            if (restsleep) {
              Watchdog.sleep(restsleep*1000);
              SERIALDEBUG_PRINT('*');             
            }
            SERIALDEBUG_PRINTLN("... woke up again");
 
#ifdef __AVR_ATmega32U4__
            // The following statement is required to prevent that LMIC spends another
            // couple of seconds busy waiting for some RX packets. This is only required
            // when using SLEEPMODE, because during sleep mode the Arduino timer variables
            // are not being incremented and LMIC job scheduling is based on this. 
//            timer0_overflow_count += 3E6;
             
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                extern volatile unsigned long timer0_millis;
                extern volatile unsigned long timer0_overflow_count;
                timer0_millis += SLEEP_TIME*1000;
                // timer0 uses a /64 prescaler and overflows every 256 timer ticks
                timer0_overflow_count += microsecondsToClockCycles((uint32_t)SLEEP_TIME * 1000000) / (64 * 256);
            }
#endif
*/
            // We need to reset the duty cycle limits within the LMIC library.
            // The reason is that in sleep mode the Arduino system timers millis and micros
            // do not get incremented. However, LMIC monitors the adherence to the
            // LoRaWAN duty cycle limitations using the system timers millis and micros.
            // Since LMIC does not know that we have slept for a long time and duty
            // cycle requirements in fact are met, we must reset the respective LMIC timers
            // in order to prevent the library to wait for some extra time (which would
            // not use sleep mode and, thus, would waste battery energy).
            LMIC.bands[BAND_MILLI].avail = os_getTime();
            LMIC.bands[BAND_CENTI].avail = os_getTime();
            LMIC.bands[BAND_DECI].avail = os_getTime();
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
//        Watchdog.sleep(MEASURE_TIME * 1000UL);
        doSleep(MEASURE_TIME * 1000UL);
#else
        delay(MEASURE_TIME * 1000UL);
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
 
            int16_temperature = round(100.0*temperature);
            int16_humidity = round(100.0*humidity);
            mydata[0] = (byte) (int16_temperature >> 8);
            mydata[1] = (byte) (int16_temperature & 0x00FF);
            mydata[2] = (byte) (int16_humidity >> 8);
            mydata[3] = (byte) (int16_humidity & 0x00FF);
 
            measuredvbat = analogRead(VBATPIN);
            measuredvbat *= 2.0;      // we divided by 2, so multiply back
            measuredvbat *= 3.3;      // Multiply by 3.3V, our reference voltage
            measuredvbat /= 1023.0;   // convert to voltage
            int16_vbat = round(measuredvbat * 100.0);
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
 
void doSleep(uint32_t time) {
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
 
  while (time > 0) {
    uint16_t slept;
    if (time < 8000)
      slept = Watchdog.sleep(time);
    else
      slept = Watchdog.sleep(8000);
 
    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += slept;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000) / (64 * 256);
    }
 
    if (slept >= time)
      break;
    time -= slept;
  }
 
  power_adc_enable();
  ADCSRA |= (1 << ADEN);
}
 
void beep(bool longbeep) {
    digitalWrite(BUZZERPIN, HIGH);   // turn the BUZZER off by making the voltage LOW
    if (longbeep)
      delay(250);
    else
      delay(100);
    digitalWrite(BUZZERPIN, LOW);   // turn the BUZZER off by making the voltage LOW
    delay(100);
}
 
void messagebeep() {
    beep(false);
    beep(true);
    beep(false);
    beep(false);
    delay(200);
    beep(false);
    beep(true);
    beep(false);
    beep(false);
}
 
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
 
    pinMode(BUZZERPIN, OUTPUT);
    digitalWrite(BUZZERPIN, LOW);   // turn the BUZZER off by making the voltage LOW
     
    alpha4.begin(0x70);             // pass in the I2C address of the display
    alpha4.clear();
    alpha4.writeDisplay();
    alpha4.writeDigitAscii(0, 'T');
    alpha4.writeDigitAscii(1, 'e');
    alpha4.writeDigitAscii(2, 's');
    alpha4.writeDigitAscii(3, 't');
    alpha4.writeDisplay();
 
    messagebeep();
 
    delay(10000);                    // give enough time to open serial monitor (if needed) or to start uploading of a new sketch
 
    alpha4.clear();
    alpha4.writeDisplay();
 
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
//    LMIC_setLinkCheckMode(0);
 
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
 
    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF9,14);
 
    // Start job. This will initiate the repetitive sending of data packets,
    // because after each data transmission, a delayed call to "do_send()"
    // is being scheduled again.
    do_send(&sendjob);
 
    // The following settings should further reduce energy consumption. I have not 
    // tested them yet, they are taken from a post in the TTN forum. See
    // https://www.thethingsnetwork.org/forum/t/adafruit-lora-feather-gateway/2440/50
    /*
    power_adc_disable();
    power_usart0_disable();
    power_twi_disable();
    power_timer1_disable();
    power_timer2_disable();
    power_timer3_disable();
    power_usart1_disable();
    power_usb_disable();
    USBCON |= (1 << FRZCLK);
    PLLCSR &= ~(1 << PLLE);
    USBCON &= ~(1 << USBE );
    clock_prescale_set(clock_div_2);
    */
}
 
void loop() {
    os_runloop_once();
}
