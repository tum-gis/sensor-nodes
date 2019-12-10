/*******************************************************************************
 * Arduino Sketch for a LoRaWAN sensor node that is registered with
 * 'The Things Network' (TTN) www.thethingsnetwork.org
 *
 * Filename: Seeeduino_LoRaWAN_GPS_BME280_OTAA_Sleep_Adafruit_V2.ino
 *
 * Author:  Thomas H. Kolbe, thomas.kolbe@tum.de
 * Version: 1.0.1
 * Last update: 2019-04-17
 *
 * This sketch works with a Seeeduino LoRaWAN microcontroller board (with or
 * without embedded GPS module). See http://wiki.seeedstudio.com/Seeeduino_LoRAWAN/
 * It requires a Seeed Grove BME280 air temperature, relative humidity,
 * and air pressure sensor module attached to the I2C Grove connector of
 * the microcontroller board. The current configuration assumes that
 * the BME280 is configured to I2C device address 0x76 (default).
 * The sketch makes a connection to The Things Network (TTN) using
 * LoRaWAN in OTAA mode. It then sends a data packet of 10 bytes to
 * LoRaWAN port 33 around every 5 minutes. The packet contains the
 * following 5 integer values (16 bit, most significant byte (MSB) first):
 *   1. temperature in Celsius (signed, multiplied by 100)
 *   2. relative humidity in percent (unsigned, multiplied by 100)
 *   3. air pressure in Pascal (unsigned, divided by 10)
 *   4. current altitude in Meters (unsigned, multiplied by 10)
 *   5. battery voltage in millivolt (unsigned)
 * These values have to be decoded by the LoRaWAN network controller
 * using a proper "payload decoder function" written in Javascript.
 *
 * Note that when the board is powered over the USB connector and
 * no battery is connected, the measured battery voltage is incorrect.
 *
 * If the board shall be running on a lithium polymer (LiPo) battery,
 * it is recommended to remove the green power LED from the board or
 * to cut the connection between the LED and the resistor lying above
 * of it as the LED constantly draws around 8mW of power. In order to
 * save energy the sketch puts the GPS module on the board to standby
 * mode right from the beginning. After each measurement and data transfer
 * the LoRaWAN module and the sensor is put to standby mode, too, and the
 * microcontroller goes into deep sleep mode. All components require
 * a total current of around 0.34mA during sleep mode and up to 65mA
 * during LoRa transmission for the board version with GPS. The board
 * version without GPS only requires 0.06mA during sleep mode. Since the
 * entire system is mostly sleeping, the GPS board should be running
 * around 2 years on a 6600mAh LiPo battery before recharging
 * (6600mAh / 0.34mA / 24 = 808 days). The non GPS board version should
 * even run for more than 10 years...
 *
 * This code is based on example code given on the Seeeduino LoRaWAN
 * wiki page. It utilizes the Open Source libraries "Adafruit_BME280"
 * and "Adafruit_Sensor" provided by the company Adafruit and the
 * library "LoRaWan.h" provided by Seeed Studio.
 *******************************************************************************/
 
#include <RTCZero.h>
#include <LoRaWan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
 
// Keep the following line, if the board is a Seeeduino LoRaWAN with GPS,
// otherwise comment the line out
 
#define HAS_GPS 1
 
#define BME280_ADDRESS       (0x76)   // I2C device address of the BME280 sensor
 
// The barometer of the BME280 can also be used to estimate the current
// altitude of the device, if the air pressure at sea level (NN) is known.
// The following value has to be set to the current air pressure at NN (in hPa)
// in order to give reasonable altitude estimations. Note that this value is
// slowly changing over time. For Munich the current value can be obtained
// from https://www.meteo.physik.uni-muenchen.de/mesomikro/stadt/messung.php
 
#define SEALEVELPRESSURE_HPA (1017.8) 
 
Adafruit_BME280 bme280;
 
RTCZero rtc;
 
unsigned char data[10];                 // buffer for the LoRaWAN data packet to be transferred
char buffer[256];                       // buffer for text messages received from the LoRaWAN module for display
 
 
void setup(void)
{
    digitalWrite(38, HIGH);             // Provide power to the 4 Grove connectors of the board
     
    for(int i = 0; i < 26; i ++)        // Set all pins to HIGH to save power (reduces the
    {                                   // current drawn during deep sleep by around 0.7mA).
        if (i!=13) {                    // Don't switch on the onboard user LED (pin 13).
          pinMode(i, OUTPUT);
          digitalWrite(i, HIGH);
        }
    }   
     
    delay(5000);                        // Wait 5 secs after reset/booting to give time for potential upload
                                        // of a new sketch (sketches cannot be uploaded when in sleep mode)
    SerialUSB.begin(115200);            // Initialize USB/serial connection
    delay(500);
    // while(!SerialUSB);
    SerialUSB.println("Seeeduino LoRaWAN board started!");
 
    if(!bme280.begin(BME280_ADDRESS)) { // Initialize the BME280 sensor module
      SerialUSB.println("BME280 device error!");
    }
 
    // Set the BME280 to a very low power operation mode (c.f. chapter 3.5
    // "Recommended modes of operation" in the BME280 datasheet. See
    // https://cdn-shop.adafruit.com/datasheets/BST-BME280_DS001-10.pdf );
    // proper values can only be queried every 60s
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X16,  // temperature
                    Adafruit_BME280::SAMPLING_X16,  // pressure
                    Adafruit_BME280::SAMPLING_X16,  // humidity
                    Adafruit_BME280::FILTER_OFF   );
 
    // nrgSave.begin(WAKE_RTC_ALARM);
    // rtc.begin(TIME_H24);
 
#ifdef HAS_GPS
    Serial.begin(9600);                 // Initialize serial connection to the GPS module
    delay(500);
    Serial.write("$PMTK161,0*28\r\n");  // Switch GPS module to standby mode as we don't use it in this sketch
#endif
     
    lora.init();                        // Initialize the LoRaWAN module
     
    memset(buffer, 0, 256);             // clear text buffer
    lora.getVersion(buffer, 256, 1);   
    memset(buffer, 0, 256);             // We call getVersion() two times, because after a reset the LoRaWAN module can be
    lora.getVersion(buffer, 256, 1);    // in sleep mode and then the first call only wakes it up and will not be performed.
    SerialUSB.print(buffer);
     
    memset(buffer, 0, 256);
    lora.getId(buffer, 256, 1);
    SerialUSB.print(buffer);
 
    // The following three constants (AppEUI, DevEUI, AppKey) must be changed
    // for every new sensor node. We are using the LoRaWAN OTAA mode (over the
    // air activation). Each sensor node must be manually registered in the
    // TTN console at https://console.thethingsnetwork.org before it can be
    // started. In the TTN console create a new device with the DevEUI also
    // being automatically generated. After the registration of the device the
    // three values can be copied from the TTN console. A detailed explanation
    // of these steps is given in
    // https://learn.adafruit.com/the-things-network-for-feather?view=all
 
    // The EUIs and the AppKey must be given in big-endian format, i.e. the
    // most-significant-byte comes first (as displayed in the TTN console).
    // For TTN issued AppEUIs the first bytes should be 0x70, 0xB3, 0xD5.
 
    // void setId(char *DevAddr, char *DevEUI, char *AppEUI);
    lora.setId(NULL, "xxxxxxxxxxxxxxxx", "yyyyyyyyyyyyyyyy");   
 
    // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
    lora.setKey(NULL, NULL, "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
     
    lora.setDeciveMode(LWOTAA);           // select OTAA join mode (note that setDeciveMode is not a typo; it is misspelled in the library)
    // lora.setDataRate(DR5, EU868);         // SF7, 125 kbps (highest data rate)
    lora.setDataRate(DR3, EU868);         // SF9, 125 kbps (medium data rate and range)
    // lora.setDataRate(DR0, EU868);         // SF12, 125 kbps (lowest data rate, highest max. distance)
 
    // lora.setAdaptiveDataRate(false);   
    lora.setAdaptiveDataRate(true);       // automatically adapt the data rate
     
    lora.setChannel(0, 868.1);
    lora.setChannel(1, 868.3);
    lora.setChannel(2, 868.5);
    lora.setChannel(3, 867.1);
    lora.setChannel(4, 867.3);
    lora.setChannel(5, 867.5);
    lora.setChannel(6, 867.7);
    lora.setChannel(7, 867.9);
 
    // The following two commands can be left commented out;
    // TTN works with the default values. (It also works when
    // uncommenting the commands, though.)
    // lora.setReceiceWindowFirst(0, 868.1);
    // lora.setReceiceWindowSecond(869.525, DR0);
    
    lora.setDutyCycle(false);             // for debugging purposes only - should normally be activated
    lora.setJoinDutyCycle(false);         // for debugging purposes only - should normally be activated
     
    lora.setPower(14);                    // LoRa transceiver power (14 is the maximum for the 868 MHz band)
     
    // while(!lora.setOTAAJoin(JOIN));
    while(!lora.setOTAAJoin(JOIN,20));    // wait until the node has successfully joined TTN
 
    lora.setPort(33);                     // all data packets are sent to LoRaWAN port 33
}
 
void loop(void)
{  
    bool result = false;   
    float temperature, altitude, pressure, humidity;
    int16_t int16_temperature, int16_humidity, int16_pressure, int16_altitude, int16_vbat;
 
    bme280.takeForcedMeasurement();       // wake the sensor up for the next readings
 
    //get and print temperatures
    SerialUSB.print("Temp: ");
    SerialUSB.print(temperature = bme280.readTemperature());
    SerialUSB.print("C  ");
   
    //get and print atmospheric pressure data
    SerialUSB.print("Pressure: ");
    SerialUSB.print(pressure = bme280.readPressure());
    SerialUSB.print("Pa  ");
 
    //get and print altitude data
    SerialUSB.print("Altitude: ");
    SerialUSB.print(altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA));
    SerialUSB.print("m  ");
 
    //get and print humidity data
    SerialUSB.print("Humidity: ");
    SerialUSB.print(humidity = bme280.readHumidity());
    SerialUSB.print("%  ");
     
    //get and print battery voltage
    SerialUSB.print("VBat: ");
    SerialUSB.print(int16_vbat=lora.getBatteryVoltage());
    SerialUSB.println("mV");
 
    int16_temperature = temperature*100.0;
    int16_humidity = humidity*100.0;
    int16_pressure = pressure/10.0;
    int16_altitude = altitude*10.0;
 
    data[0] = (byte) (int16_temperature >> 8);
    data[1] = (byte) (int16_temperature & 0x00FF);
    data[2] = (byte) (int16_humidity >> 8);
    data[3] = (byte) (int16_humidity & 0x00FF);
    data[4] = (byte) (int16_pressure >> 8);
    data[5] = (byte) (int16_pressure & 0x00FF);
    data[6] = (byte) (int16_altitude >> 8);
    data[7] = (byte) (int16_altitude & 0x00FF);
    data[8] = (byte) (int16_vbat >> 8);
    data[9] = (byte) (int16_vbat & 0x00FF);
     
    result = lora.transferPacket(data, 10, 5);   // send the data packet (10 bytes) with a default timeout of 5 secs
     
    if(result)
    {
        short length;
        short rssi;
         
        memset(buffer, 0, 256);
        length = lora.receivePacket(buffer, 256, &rssi);
         
        if(length)
        {
            SerialUSB.print("Length is: ");
            SerialUSB.println(length);
            SerialUSB.print("RSSI is: ");
            SerialUSB.println(rssi);
            SerialUSB.print("Data is: ");
            for(unsigned char i = 0; i < length; i ++)
            {
                SerialUSB.print("0x");
                SerialUSB.print(buffer[i], HEX);
                SerialUSB.print(" ");
            }
            SerialUSB.println();
        }
    }
     
    lora.setDeviceLowPower();     // bring the LoRaWAN module to sleep mode
    doSleep((5*60-8)*1000);       // deep sleep for 292 secs (+ 3 secs transmission time + 5 secs timeout = 300 secs period)
    lora.setPort(33);             // send some command to wake up the LoRaWAN module again
}
 
// The following function implements deep sleep waiting. When being called the
// CPU goes into deep sleep mode (for power saving). It is woken up again by
// the CPU-internal real time clock (RTC) after the configured time.
//
// A similar function would also be available in the standard "ArduinoLowPower" library.
// However, in order to be able to use that library with the Seeeduino LoRaWAN board,
// four files in the package "Seeed SAMD boards by Seeed Studio Version 1.3.0" that is
// installed using the Arduino IDE board manager need to be patched. The reason is that
// Seeed Studio have not updated their files to a recent Arduino SAMD version yet
// and the official "ArduinoLowPower" library provided by the Arduino foundation is
// referring to some missing functions. For further information see here:
// https://forum.arduino.cc/index.php?topic=603900.0 and here:
// https://github.com/arduino/ArduinoCore-samd/commit/b9ac48c782ca4b82ffd7e65bf2c956152386d82b
 
void doSleep(uint32_t millis) {
    if (!rtc.isConfigured()) {    // if called for the first time,
        rtc.begin(false);         // then initialize the real time clock (RTC)
    }
 
    uint32_t now = rtc.getEpoch();
    rtc.setAlarmEpoch(now + millis/1000);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
 
    rtc.standbyMode();            // bring CPU into deep sleep mode (until woken up by the RTC)
}