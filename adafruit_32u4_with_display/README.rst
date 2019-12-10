Adafruit 32u4 LoRa with Display
===============================

This tutorial is made to showcase the use of Adafruit 32u4 board to create
a LoRaWAN enabled sensor node with a display and a case. In the following
example, a temperature and humidity sensor was used with the Adafruit 32u4
board to create this tutorial.

Hardware
--------

To build this sensor node we have used following hardware components:

- `Adafruit Feather 32u4 LoRa module <https://www.adafruit.com/product/3078>`_
- `Grove - DHT-22 Temperature & Humidity Sensor <http://wiki.seeedstudio.com/Grove-Temperature_and_Humidity_Sensor_Pro/>`_
- `LED Display <https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing>`_
- `Breadboard <https://en.wikipedia.org/wiki/Breadboard#/media/File:400_points_breadboard.jpg>`_
- `Battery <https://www.adafruit.com/product/2011>`_
- `Resistor: 4.7k to 10k Ohm <https://learn.sparkfun.com/tutorials/resistors/all>`_
- `3d-Printed case <https://www.thingiverse.com/thing:2209964>`_

Microcontroller
^^^^^^^^^^^^^^^

The Adafruit Feather 32u4 LoRa module is operated by the 8bit ATmega32u4
microcontroller running at 8MHz. It has 32 KB flash memory (to store the
program code) and 2 KB of RAM (to store variables, status information,
and buffers). The operating voltage of the board is 3.3V (this is important
when attaching sensors and other peripherals; they also must operate on 3.3V).
The board offers 20 general purpose digital input/output pins (20 GPIOs)
with 10 analog input pins (with 12bit analog digital converters (ADC)),
one serial port (programmable Universal Asynchronous Receiver and
Transmitter, UART), one I2C port, one SPI port, one USB port. The board
comes with an embedded Lithium polymer battery management chip and status
indicator led, which allows to directly connect a 3.7V LiPo rechargeable
battery that will be automatically recharged when the board is powered over
its USB connector. The Adafruit Feather 32u4 LoRa board is available in
German shops from around 37 € to 45 €.

The LoRa transmitter and receiver is encapsulated within an RFM95 module
from the company HopeRF. This module uses the LoRa chip SX1276 from the
company Semtech and is dedicated to the 868 MHz frequency band. The RFM95
module is connected via SPI interface to the microcontroller. Most of the
required connections of the LoRa transceiver pins with the microcontroller
are already built-in on the Adafruit Feather 32u4 LoRa board. However,
Digital Pin 6 of the microcontroller must be connected to DIO1 of the LoRa
transceiver module in addition using a simple wire. Since the module only
implements the LoRa physical layer, the LoRaWAN protocol stack must be
implemented in software on the microcontroller. We are using the Arduino
library LMIC for that purpose (see below). The implemented LoRaWAN
functionality is compatible with LoRaWAN Class A/C.

.. figure:: 32u4board.jpg
  :width: 70 %
  :align: center

  `Feather 32u4 with RFM95 LoRa Radio-868 MHz-RadioFruit <https://www.adafruit.com/product/3078>`_
  from Adafruit. `Feather 32u4 LoRa tutorial with explanations, datasheets, and
  examples <https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/>`_.

Sensor
^^^^^^

We have attached a DHT22 sensor to the microcontroller board, which
measures air temperature and humidity. The minimal time interval between
two measurements is 2 seconds. All data transfers between the DHT22 and
the microcontroller use a single digital line. The sensor data pin is
attached to a GPIO pin (here: Digital Pin 6) of the microcontroller. In
addition, a so-called pull-up resistor of 4.7k to 10k Ohm must be connected
between the data line and VCC (+3.3V). The
`DHT22 datasheet <https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf>`_
provides more technical details about the DHT22 Sensor. A tutorial on
`how to use the DHT22 sensor with Arduino <https://learn.adafruit.com/dht?view=all>`_
microcontrollers is provided here. The sensor is available in German shops
for around 4 € to 10 €.

.. figure:: setup.jpg
  :width: 100 %
  :align: center

  The Adafruit Feather 32u4 RFM95 LoRa installed in a 3D printed case. On
  top of the microcontroller board an Adafruit Display Wing with a 4 digit
  14 segments LED display is attached. Right of the display the DHT22
  temperature / humidity sensor is mounted. In the rear part of the case
  a 2000 mAh polymer (LiPo) battery is installed. On the right side the
  antenna is visible.

Display / Beeper
^^^^^^^^^^^^^^^^

On top of the microcontroller board we have attached an `Adafruit Display
Wing with a 4 digit 14 segments LED display <https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing>`_.
It can show 0-4 numbers or letters (upper case and lower case). The
display controller is using the `I2C protocol <https://en.wikipedia.org/wiki/I%C2%B2C>`_
and the I2C pins SDA and SCL are directly connected to the Adafruit Feather
via the Wing connectors. The Wing is using the default I2C address (0x70).
Also a 3.3V beeper is installed that is used to indicate that a new message
was received and is now being displayed. The '+' pin of the beeper has to be
connected to Digital Pin 12 and the '-' pin to GND. The display and the beeper
can be used to notify a user with (very) short messages. The reason why we
have included this is mostly to experiment with and to demonstrate the downlink
capabilities of LoRaWAN. When a downlink message has been queued it will be
transmitted to the node right after it has transmitted the next data packet
(uplink data). Hence, it depends on the transmission time period how long
it can take unless the node receives and displays a downlink message.

Case
^^^^

The case was 3D printed using the
`design files provided by Adafruit <https://learn.adafruit.com/3d-printed-case-for-adafruit-feather/overview>`_.
The case consists of three parts. Part 1 is the main enclosure (it does not
have a switch holder or tabs, the design file is feather-case.stl). Part 2
is the battery holder (with a slide switch holder, the design file is
feather-bat-switch.stl). Part 3 is the case topper (with a cutout for the
Adafruit Feather Wing, the design file is feather-top-wing.stl). All design
files can be downloaded from `Thingiverse <https://www.thingiverse.com/thing:2209964>`_.

We have ordered the three parts from an online 3D printing service. The
quality of the delivered parts was generally ok, but not good enough for
snapping the three parts firmly together. It is not clear yet whether this
is a problem of the design files or of the printing service. We used two
rubber bands In order to fix the three parts together.

Once all these connection are made, the board is connected with a computer
using a USB cable. Further, steps of `software part <#software>`_ needs to
be followed. But, before that we need to `register a new device on the service
<#registration-of-the-sensor-node-with-the-things-network-ttn>`_
that we are using.

Software
--------

The sensor node has been programmed using the
`Arduino IDE <https://www.arduino.cc/en/main/software>`_. Please note, that
in the Arduino framework a program is called a 'Sketch'.

After the sketch has successfully established a connection to The Things
Network it reports the air temperature, humidity, and the voltage of a (possibly)
attached LiPo battery every 5 minutes. All three values are being encoded
in two byte integer values each (in most significant byte order) and then
sent as a 6 bytes data packet to the respective TTN application using LoRaWAN
port 7. Please note, that LoRaWAN messages can be addressed to ports 1-255
(port 0 is reserved); these ports are similar to port numbers 0-65535 when
using the Internet TCP/IP protocol. Voltage and humidity values are always
greater or equal to 0, but the temperature value can also become negative.
Negative values are represented as a `two's complement <https://en.wikipedia.org/wiki/Two%27s_complement>`_;
this must be considered in the Payload Decoding Function used in The
Things Network (`see here <#ttn-payload-decoding>`_).

In between two sensor readings the microcontroller is going into deep sleep
mode to save battery power. We still have to run some tests to find out for
how long the system can run using the 2000 mAh LiPo battery and the current
version of the sketch. Showing a received message on the display draws a
considerable amount of power and will shorten battery life significantly.
Hence, when running on battery it is recommended to clear a displayed
message soon by sending a simple space character (0x20). (Further
optimizations would be possible, for example, not switching on the LED on
the microcontroller board during LoRa data transmissions.)

The employed RFM95 LoRa module does not provide built-in support of the
LoRaWAN protocol. Thus, it has to be implemented on the ATmega32u4
microcontroller. We use the `IBM LMIC (LoraMAC-in-C) library
<https://github.com/matthijskooijman/arduino-lmic>`_ for Arduino.
Since the ATmega32u4 microcontroller only has 32 KB of flash memory and
the LMIC library is taking most of it, there is only very limited code
space left for the application dealing with the sensors (about 2 KB).
Nevertheless, this is sufficient to query some sensors like in our
example the DHT22.

Now download and run the :ref:`Arduino_Sketch_Adafruit_32u4-Display.ino`
file in the Arduino IDE. This code was created by merging the example
code of both the sensors and the ttn-otaa example from the lmic library.
Some required changes were made while merging the example codes. The user
should change the network session key, app session key and device address
in the code before compiling. These keys can be obtained from the TTN
account as shown in the `services section <#services>`_.

.. literalinclude:: Arduino_Sketch_Adafruit_32u4-Display/Arduino_Sketch_Adafruit_32u4-Display.ino
   :language: arduino
   :linenos:
   :caption: Modify the keys in highlighted lines.
   :name: Arduino_Sketch_Adafruit_32u4-Display.ino_Keys
   :lines: 189-204
   :emphasize-lines: 4,8,15

Following is the example code that can be used to measure the battery
voltage of the sensor node:

.. literalinclude:: Arduino_Sketch_Adafruit_32u4-Display/Arduino_Sketch_Adafruit_32u4-Display.ino
   :language: arduino
   :linenos:
   :caption: Code for measuring the battery voltage
   :name: Arduino_Sketch_Adafruit_32u4-Display.ino_BatteryVoltage
   :lines: 441-451

Services
--------

The services used for this sensor-node are:

- `TheThingsNetwork <#registration-of-the-sensor-node-with-the-things-network-ttn>`_ service for LoRaWAN network service.
- `TheThingsNetwork - OGC SensorWeb <#the-things-network-ogc-sensorweb-integration>`_ integration for uploading LoRaWAN sensor data into OGC infrastructure.

Registration of the sensor node with The Things Network (TTN)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The LoRaWAN protocol makes use of a number of different identifiers,
addresses, keys, etc. These are required to unambiguously identify
devices, applications, as well as to encrypt and decrypt messages.
The names and meanings are `nicely explained on a dedicated TTN web
page <https://www.thethingsnetwork.org/docs/lorawan/addressing.html>`_.

The sketch given above connects the sensor node with The Things Network
(TTN) using the Over-the-Air-Activation (OTAA) mode. In this mode, we
use the three keys AppEUI, DevEUI, AppKey. The DevEUI should be
delivered with the sensor node by the manufacturer, the other two
keys are created using the TTN console. Each sensor node must be
manually registered in the `TTN console <https://console.thethingsnetwork.org>`_
before it can be started. This assumes that you already have a TTN
user account (which needs to be created otherwise). `In the TTN console
create a new device <https://www.thethingsnetwork.org/docs/devices/registration.html>`_
and enter the DevEUI number that was shipped with the Adafruit Feather
LoRa board. Note that the shipped number only consists of 6 bytes
while LoRaWAN requires an 8 bytes DevEUI. We simply add 0x00 0x00
in the middle of the 6 bytes provided. If you have lost the provided
DevEUI you can also let the TTN console create a new one. After the
registration of the device the respective keys (AppEUI, DevEUI, AppKey)
can be copied from the TTN console and must be pasted into the the
proper places in the source code of the sketch above. Please make
sure that you choose for each of the three keys the correct byte
ordering (DevEUI, AppEUI in LSB; AppKey in MSB). A detailed explanation
of these steps is `given here <https://learn.adafruit.com/the-things-network-for-feather?view=all>`_.
Then the sketch can be compiled and uploaded to the Adafruit Feather
32u4 LoRa microcontroller. Note that the three constants (AppEUI, DevEUI,
AppKey) must be changed in the source code for every new sensor node.

Using the OTAA mode has the advantage over the ABP (activation by
personalization) mode that during connection the session keys are newly
created which improves security. Another advantage is that the packet
counter is automatically reset to 0 both in the node and in the TTN
application.

TTN Payload Decoding
^^^^^^^^^^^^^^^^^^^^

Everytime a data packet is received by a TTN application a dedicated
Javascript function is being called (Payload Decoder Function). This
function can be used to decode the received byte string and to create
proper Javascript objects or values that can directly be read by humans
when looking at the incoming data packet. This is also useful to format
the data in a specific way that can then be forwarded to an external
application (e.g. a sensor data platform like `MyDevices <https://mydevices.com/>`_
or `Thingspeak <https://thingspeak.com/>`_). Such a forwarding can be
configured in the TTN console in the "Integrations" tab. :ref:`TTN_Payload_Decode.js`
given here checks if a packet was received on LoRaWAN port 7 and then
assumes that it consists of the 6 bytes encoded as described above.
It creates the three Javascript objects 'temperature', 'humidity',
and 'vbattery'. Each object has two fields: 'value' holds the value
and 'uom' gives the unit of measure. The source code can simply be
copied and pasted into the 'decoder' tab in the TTN console after
having selected the application. Choose the option 'Custom' in the
'Payload Format' field. Note that when you also want to handle other
sensor nodes sending packets on different LoRaWAN ports, then the
Payload Decoder Function can be extended after the end of the  if
(port==7) {...} statement by adding  else if (port==8) {...} else
if (port==9) {...} etc.

The Things Network - OGC SensorWeb Integration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The presented Payload Decoder Function works also with the
`TTN-OGC SWE Integration <https://github.com/52North/ttn-ogcswe-integration>`_
for the `52° North Sensor Observation Service (SOS) <https://github.com/52North/SOS>`_.
This software component can be downloaded from
`this repository <https://github.com/52North/ttn-ogcswe-integration>`_.
It connects a TTN application with a running transactional
`Sensor Observation Service 2.0.0 (SOS) <https://www.opengeospatial.org/standards/sos>`_.
Data packets received from TTN are imported into the SOS. The SOS
persistently stores sensor data from an arbitrary number of sensor nodes
and can be queried for the most recent as well as for historic sensor
data readings. The 52° North SOS comes with its own REST API and a
nice web client allowing to browse the stored sensor data in a
convenient way.

We are running an instance of the 52° North SOS and the TTN-OGC
SWE Integration. The web client for this LoRaWAN sensor node can
be accessed `on this page <http://129.187.38.201:8080/ttn-sos-integration/static/client/helgoland/index.html#/diagram?ts=ttnOGC__30,ttnOGC__29,ttnOGC__28>`_.
Here is a screenshot showing the webclient:

.. figure:: webclient.png
  :width: 100 %
  :align: center

  Web client for data visualization

Sending a message to the Sensor Node (Downlink)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Using the TTN console we can send a message (i.e. a byte string)
to the sensor node. In the `TTN console application page <https://console.thethingsnetwork.org/applications>`_
click on the respective application. Then click on the 'Devices'
tab and choose the proper sensor node (here: adafruit-feather-32u4-lora3).
On the overview page scroll down to the 'Downlink' section. In the 'Payload'
field enter 1 to 4 bytes. In order to show digits or letters on the LED
display these must be `ASCII encoded <https://en.wikipedia.org/wiki/ASCII>`_
and have to be entered as hexadecimal numbers. When you click on the
'Send' button the message will be queued and the next time when the
node sends its data packet (uplink) it will receive the message. The
first 4 bytes will be shown on the display and the beeper indicates
the reception of a new downlink message. In order to blank the display
just send a one byte message with the value '20' (hexadecimal for 32,
which is the ASCII code for a space). When the node receives just a
single blank character it will not produce a beeping sound. There is
a nice `web page <https://www.rapidtables.com/convert/number/ascii-hex-bin-dec-converter.html>`_
offering online encoding of text to ASCII numbers in hexadecimal encoding.
For example, in order to display the text 'LoRa', the four hexadecimal
numbers 4C 6F 52 61 have to be entered in the Payload entry field.

Code files
----------

.. literalinclude:: Arduino_Sketch_Adafruit_32u4-Display/Arduino_Sketch_Adafruit_32u4-Display.ino
   :language: arduino
   :linenos:
   :caption: Arduino Sketch for Adafruit32u4 LoRa with display sensor node
   :name: Arduino_Sketch_Adafruit_32u4-Display.ino

.. literalinclude:: TTN_Payload_Decode.js
   :language: Javascript
   :linenos:
   :caption: TTN payload decoder for Adafruit32u4 LoRa with display sensor node
   :name: TTN_Payload_Decode.js

References
----------

- `Adafruit Feather 32u4 LoRa microntroller <https://www.adafruit.com/product/3078>`_
- `Adafruit Feather 32u4 LoRa tutorial <https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/>`_
- `IBM LMIC (LoraMAC-in-C) library for Arduino <https://github.com/matthijskooijman/arduino-lmic>`_
- `Using Adafruit Feather 32u4 RFM95 as an TTN Node - Stories - Labs <https://www.thethingsnetwork.org/labs/story/using-adafruit-feather-32u4-rfm95-as-an-ttn-node>`_
- `TTN LoraWan Atmega32U4 based node – ABP version | Primal Cortex's Weblog <https://primalcortex.wordpress.com/2017/10/31/ttnlorawan32u4node/>`_
- `node-workshop/lora32u4.md at master · kersing/node-workshop · GitHub <https://github.com/kersing/node-workshop/blob/master/lora32u4.md>`_
- `Got Adafruit Feather 32u4 LoRa Radio to work and here is how - End Devices (Nodes) - The Things Network <https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863/35>`_
- `Adafruit Feather as LoRaWAN node | Wolfgang Klenk <https://wolfgangklenk.wordpress.com/2017/04/15/adafruit-feather-as-lorawan-node/>`_
- `LMiC on Adafruit Lora Feather successfully sends message to TTN and then halts with "Packet queued" - End Devices (Nodes) - The Things Network <https://www.thethingsnetwork.org/forum/t/lmic-on-adafruit-lora-feather-successfully-sends-message-to-ttn-and-then-halts-with-packet-queued/3762/25>`_
- `GitHub - marcuscbehrens/loralife <https://github.com/marcuscbehrens/loralife>`_
- `GPS-Tracker - Stories - Labs <https://www.thethingsnetwork.org/labs/story/gps-tracker>`_

**On battery saving / using the deep sleep mode**

- `Adafruit Feather 32u4 LoRa - long transmission time after deep sleep - End Devices (Nodes) - The Things Network <https://www.thethingsnetwork.org/forum/t/adafruit-feather-32u4-lora-long-transmission-time-after-deep-sleep/11678/7>`_ and `this <https://www.thethingsnetwork.org/forum/t/adafruit-feather-32u4-lora-long-transmission-time-after-deep-sleep/11678/13>`_
- `Full Arduino Mini LoraWAN and 1.3uA Sleep Mode - End Devices (Nodes) - The Things Network <https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-below-1ua-sleep-mode/8059/97>`_
- `Adding Method to Adjust hal_ticks Upon Waking Up from Sleep · Issue #109 · matthijskooijman/arduino-lmic <https://github.com/matthijskooijman/arduino-lmic/issues/109>`_
- `minilora-test/minilora-test.ino at cbe686826bd84fac8381de47b5f5b02dd47c2ca0 · tkerby/minilora-test <https://github.com/tkerby/minilora-test/blob/cbe686826bd84fac8381de47b5f5b02dd47c2ca0/minilora-test/minilora-test.ino#L190>`_
- `Arduino-LMIC library with low power mode - Mario Zwiers <https://mariozwiers.de/2018/04/04/arduino-lmic-library-with-low-power-mode/>`_
