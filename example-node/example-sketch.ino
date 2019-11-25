#include <DHT.h>

DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  // setup
}

void loop()
{
  // loop
}

/**
 * Blink the builtin LED
 * 
 * int ms     turn LED on for n millis
**/
void blink(int ms)
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(ms);
  digitalWrite(LED_BUILTIN, LOW);
}
