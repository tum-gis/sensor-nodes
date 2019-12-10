#include <DHT.h>
#include <RTCZero.h>
#include <LoRaWan.h>
#include <Wire.h>
#include <CayenneLPP.h>
 
// Keep the following line, if the board is a Seeeduino LoRaWAN with GPS,
// otherwise comment the line out
 
// #define HAS_GPS 1

const int analogInPin = A0;
#define DHTPIN A2   
#define DHTTYPE DHT22 

DHT dht(DHTPIN, DHTTYPE);

int BatteryValue = 0;        
float outputValue = 0;

RTCZero rtc;
char buffer[256];                       // buffer for text messages received from the LoRaWAN module for display
 
CayenneLPP lpp(51);
 
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
    dht.begin();                                    // of a new sketch (sketches cannot be uploaded when in sleep mode)
    SerialUSB.begin(115200);            // Initialize USB/serial connection
    delay(500);
    // while(!SerialUSB);
    SerialUSB.println("Seeeduino LoRaWAN board started!");
 
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
    lora.setId(NULL, "00942FBXXXXXXXXX", "70B3D57XXXXXXXXX");   
 
    // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
    lora.setKey(NULL, NULL, "CB89A0AA43F6C5XXXXXXXXXXXXXXXXXX");
     
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
    float temp_hum_val[2] = {0};
    float temperature, humidity;
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    
    
    if(!dht.readTempAndHumidity(temp_hum_val)){
        SerialUSB.print("Humidity: "); 
        SerialUSB.print(humidity = temp_hum_val[0]);
        SerialUSB.print(" %\t");
        SerialUSB.print("Temperature: "); 
        SerialUSB.print(temperature = temp_hum_val[1]);
        SerialUSB.println(" *C");
    }
    else{
       SerialUSB.println("Failed to get temprature and humidity value.");
    }
       
    BatteryValue = analogRead(analogInPin);
    // Calculate the battery voltage value
    outputValue = (float(BatteryValue)*5)/1023*2;
    // print the results to the serial monitor:
    SerialUSB.print("Analog value = " );
    SerialUSB.print(BatteryValue);
    SerialUSB.print("\t voltage = ");
    SerialUSB.println(outputValue);
    SerialUSB.println("V \n");
     
    SerialUSB.println("-- LOOP");
    lpp.reset();
    lpp.addTemperature(1, temperature);
    lpp.addRelativeHumidity(2, humidity);
    lpp.addAnalogInput(3, outputValue);
        
    result = lora.transferPacket(lpp.getBuffer(), lpp.getSize(), 5);   // send the data packet (n byts) with a default timeout of 5 secs

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
