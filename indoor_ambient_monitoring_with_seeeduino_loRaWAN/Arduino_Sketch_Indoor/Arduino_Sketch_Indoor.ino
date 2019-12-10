#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include "Air_Quality_Sensor.h"
#include "seeed_bme680.h"
#include "Seeed_HM330X.h"
#include <SoftwareSerial.h>
#include <MHZ19.h>
#include <RTCZero.h>
#include <LoRaWan.h>
#include <CayenneLPP.h>

RTCZero rtc;
char buffer[256];                       // buffer for text messages received from the LoRaWAN module for display
 
CayenneLPP lpp(51);

AirQualitySensor sensors(A2);

SoftwareSerial ss(4,5);
MHZ19 mhz(&ss);

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define IIC_ADDR  uint8_t(0x76)
Seeed_BME680 bme680(IIC_ADDR);

int loudness,a;

HM330X sensor;
u8 buf[30];

const char *str[]={"sensor num: ","PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                    "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                    "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                    "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                    "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                    "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

err_t print_result(const char* str,u16 value)
{
    if(NULL==str)
        return ERROR_PARAM;
    SerialUSB.print(str);
    SerialUSB.println(value);
    return NO_ERROR;
}

/*parse buf with 29 u8-data*/
err_t parse_result(u8 *data)
{
    u16 value=0;
    err_t NO_ERROR;
    if(NULL==data)
        return ERROR_PARAM;
    for(int i=1;i<8;i++)
    {
         value = (u16)data[i*2]<<8|data[i*2+1];
         print_result(str[i-1],value);
         if(i==6)
           {   a=value;
               SerialUSB.println(a);
           }
    }
}

err_t parse_result_value(u8 *data)
{
    if(NULL==data)
        return ERROR_PARAM;
    for(int i=0;i<28;i++)
    {
        SerialUSB.print(data[i],HEX);
        SerialUSB.print("  ");
        if((0==(i)%5)||(0==i))
        {
            SerialUSB.println(" ");
        }
    }
    u8 sum=0;
    for(int i=0;i<28;i++)
    {
        sum+=data[i];
    }
    if(sum!=data[28])
    {
        SerialUSB.println("wrong checkSum!!!!");
    }
    SerialUSB.println(" ");
    SerialUSB.println(" ");
    return NO_ERROR;
}


void setup()
{ 
  Wire.begin();
  
  for(int i = 0; i < 26; i ++)        // Set all pins to HIGH to save power (reduces the
    {                                   // current drawn during deep sleep by around 0.7mA).
        if (i!=13) {                    // Don't switch on the onboard user LED (pin 13).
          pinMode(i, OUTPUT);
          digitalWrite(i, HIGH);
        }
    }   
     
  delay(5000); 
  
  SerialUSB.begin(115200);
      delay(100);
  SerialUSB.println("SerialUSB start");

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
 
    lora.setPort(33);   
    
    if(sensor.init())
    {
        SerialUSB.println("HM330X init failed!!!");
        while(1);
    }

  if (sensors.init()) {
    SerialUSB.println("Sensor ready.");
  }
  else {
    SerialUSB.println("Sensor ERROR!");
  }
  
  TSL2561.init();

    while (!bme680.init()) 
  {
    SerialUSB.println("bme680 init failed ! can't find device!");
  delay(10000);
  }

  ss.begin(9600);

}

void loop()
{
  bool result = false;
  float temperature,humidity,pressure,airquality,light,gas,CO2;
    
  loudness = analogRead(0);
  SerialUSB.print("The Loudness Sensor value is: ");
  SerialUSB.println(loudness);
  SerialUSB.println();
  delay(3000);
    
  int quality = sensors.slope();

  SerialUSB.print("Air Quality Sensor value is: ");
  SerialUSB.println(airquality=sensors.getValue());
  
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    SerialUSB.println("High pollution! Force signal active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    SerialUSB.println("High pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    SerialUSB.println("Low pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    SerialUSB.println("Fresh air.");
  }
  SerialUSB.println();
  delay(3000);
  
  SerialUSB.print("The Light Sensor value is: ");
  SerialUSB.println(light=TSL2561.readVisibleLux());
  SerialUSB.println();
  delay(3000);
    
    if(sensor.read_sensor_value(buf,29))
    {
        SerialUSB.println("HM330X read result failed!!!");
    }
    parse_result_value(buf);
    parse_result(buf);
    SerialUSB.println(" ");
    delay(3000);

    if (bme680.read_sensor_data()) 
  {
    SerialUSB.println("Failed to perform reading :(");
    return;
  }
  SerialUSB.print("temperature ===>> ");
  SerialUSB.print(temperature = bme680.sensor_result_value.temperature);
  SerialUSB.println(" C");

  SerialUSB.print("pressure ===>> ");
  SerialUSB.print(pressure = bme680.sensor_result_value.pressure/ 1000.0);
  SerialUSB.println(" KPa");

  SerialUSB.print("humidity ===>> ");
  SerialUSB.print(humidity = bme680.sensor_result_value.humidity);
  SerialUSB.println(" %");

  SerialUSB.print("gas ===>> ");
  SerialUSB.print(gas = bme680.sensor_result_value.gas/ 1000.0);
  SerialUSB.println(" Kohms");

  SerialUSB.println();
  
  delay(3000);

  MHZ19_RESULT response = mhz.retrieveData();
  if (response == MHZ19_RESULT_OK)
  {
    SerialUSB.print(F("CO2: "));
    SerialUSB.println(CO2=mhz.getCO2());
    SerialUSB.print(F("Min CO2: "));
    SerialUSB.println(mhz.getMinCO2());
    SerialUSB.print(F("Temperature: "));
    SerialUSB.println(mhz.getTemperature());
    SerialUSB.print(F("Accuracy: "));
    SerialUSB.println(mhz.getAccuracy());
    SerialUSB.println();
  }
  else
  {
    SerialUSB.print(F("Error, code: "));
    SerialUSB.println(response);
  }
  
    lpp.reset();
    lpp.addTemperature(1, temperature);
    lpp.addRelativeHumidity(2, humidity);
    lpp.addAnalogInput(3, airquality);
    lpp.addLuminosity(4, light);
    lpp.addBarometricPressure(5, pressure);
    lpp.addLuminosity(6, CO2);
    lpp.addAnalogInput(7, gas);
    lpp.addLuminosity(8, loudness);
    lpp.addLuminosity(9, a);   
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
    lora.setPort(33);    

}

void doSleep(uint32_t millis) {
    if (!rtc.isConfigured()) {    // if called for the first time,
        rtc.begin(false);         // then initialize the real time clock (RTC)
    }
 
    uint32_t now = rtc.getEpoch();
    rtc.setAlarmEpoch(now + millis/1000);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
 
    rtc.standbyMode();            // bring CPU into deep sleep mode (until woken up by the RTC)
}
