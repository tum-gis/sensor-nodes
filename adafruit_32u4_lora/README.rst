Adafruit 32u4 LoRa
==================

This tutorial is made to showcase the use of Adafruit 32u4 board
to create a LoRaWAN enabled sensor node. In the following example,
a temperature and humidity sensor was used with the Adafruit 32u4
board.

Hardware
--------

To build this sensor node we have used following hardware components:

- `Adafruit Feather 32u4 LoRa module <https://www.adafruit.com/product/3078>`_
- `Grove - DHT-22 Temperature & Humidity Sensor <http://wiki.seeedstudio.com/Grove-Temperature_and_Humidity_Sensor_Pro/>`_
- `Breadboard <https://en.wikipedia.org/wiki/Breadboard#/media/File:400_points_breadboard.jpg>`_
- `Battery <https://www.adafruit.com/product/353>`_
- `Resistor: 4.7k to 10k Ohm <https://learn.sparkfun.com/tutorials/resistors/all>`_

Microcontroller
^^^^^^^^^^^^^^^

The Adafruit Feather 32u4 LoRa module is operated by the 8bit
ATmega32u4 microcontroller running at 8MHz. It has 32 KB flash
memory (to store the program code) and 2 KB of RAM (to store
variables, status information, and buffers). The operating
voltage of the board is 3.3V (this is important when attaching
sensors and other peripherals; they also must operate on 3.3V).
The board offers 20 general purpose digital input/output pins
(20 GPIOs) with 10 analog input pins (with 12bit analog digital
converters (ADC)), one serial port (programmable Universal
Asynchronous Receiver and Transmitter, UART), one I2C port,
one SPI port, one USB port. The board comes with an embedded
Lithium polymer battery management chip and status indicator
led, which allows to directly connect a 3.7V LiPo rechargeable
battery that will be automatically recharged when the board is
powered over its USB connector. The Adafruit Feather 32u4 LoRa
board is available in German shops from around 37 € to 45 €.

The LoRa transmitter and receiver is encapsulated within an
RFM95 module from the company HopeRF. This module uses the
LoRa chip SX1276 from the company Semtech and is dedicated to
the 868 MHz frequency band. The RFM95 module is connected via
SPI interface to the microcontroller. Most of the required
connections of the LoRa transceiver pins with the microcontroller
are already built-in on the Adafruit Feather 32u4 LoRa board.
However, Digital Pin 6 of the microcontroller must be connected
to DIO1 of the LoRa transceiver module in addition using a simple
wire. Since the module only implements the LoRa physical layer,
the LoRaWAN protocol stack must be implemented in software on
the microcontroller. We are using the Arduino library LMIC for
that purpose (see below). The implemented LoRaWAN functionality
is compatible with LoRaWAN Class A/C.

.. figure:: 32u4board.jpg
  :width: 70 %
  :align: center

  `Feather 32u4 with RFM95 LoRa Radio-868 MHz-RadioFruit <https://www.adafruit.com/product/3078>`_
  from Adafruit. `Feather 32u4 LoRa tutorial with explanations,
  datasheets, and examples <https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/>`_

Sensor
^^^^^^

We have attached a DHT22 sensor to the microcontroller board,
which measures air temperature and humidity. The minimal time
interval between two measurements is 2 seconds. All data
transfers between the DHT22 and the microcontroller use a
single digital line. The sensor data pin is attached to a GPIO
pin (here: Digital Pin 5) of the microcontroller. In addition,
a so-called pull-up resistor of 4.7k to 10k Ohm must be connected
between the data line and VCC (+3.3V). The `DHT22 datasheet
<https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf>`_
provides more technical details about the DHT22 Sensor. A
tutorial on how to use the `DHT22 sensor with Arduino
microcontrollers <https://learn.adafruit.com/dht?view=all>`_ is
provided here. The sensor is available in German shops for
around 4 € to 10 €.

.. figure:: setup.png
  :width: 100 %
  :align: center

  The Adafruit Feather 32u4 RFM95 LoRa with attached
  antenna (top), a 1000 mAh lithium polymer (LiPo) battery
  (bottom), and an attached DHT22 temperature / humidity
  sensor (white box on the left)

For more details on the wiring connections, follow `this
tutorial <https://github.com/tum-gis/sensor-nodes/tree/master/FeatherM0LoRa%20in%20TFA%20Housing#dht-22-sensor-connections>`_.
Once all these connection are made, the board is connected
with a computer using a USB cable. Further, steps of `software
part <#software>`_ needs to be followed. But, before that we
need to `register a new device on the service
<#registration-of-the-sensor-node-with-the-things-network-ttn>`_
that we are using.

Software
--------

The sensor node has been programmed using the `Arduino
IDE <https://www.arduino.cc/en/main/software>`_. Please note,
that in the Arduino framework a program is called a 'Sketch'.

After the sketch has successfully established a connection to
The Things Network it reports the air temperature, humidity,
and the voltage of a (possibly) attached LiPo battery every 5
minutes. All three values are being encoded in two byte integer
values each (in most significant byte order) and then sent as
a 6 bytes data packet to the respective TTN application using
LoRaWAN port 7. Please note, that LoRaWAN messages can be
addressed to ports 1-255 (port 0 is reserved); these ports are
similar to port numbers 0-65535 when using the Internet TCP/IP
protocol. Voltage and humidity values are always greater or
equal to 0, but the temperature value can also become negative.
Negative values are represented as a `two's complement
<https://en.wikipedia.org/wiki/Two%27s_complement>`_; this must
be considered in the Payload Decoding Function used in The
Things Network (`see here <#ttn-payload-decoding>`_).

In between two sensor readings the microcontroller is going
into deep sleep mode to save battery power. With a 1000 mAh
LiPo battery and the current version of the sketch the system
can run for at least 5 months. (Further optimizations would be
possible, for example, not switching on the LED on the
microcontroller board during LoRa data transmissions.)

The employed RFM95 LoRa module does not provide built-in support
of the LoRaWAN protocol. Thus, it has to be implemented on the
ATmega32u4 microcontroller. We use the `IBM LMIC (LoraMAC-in-C)
library <https://github.com/matthijskooijman/arduino-lmic>`_
for Arduino. Since the ATmega32u4 microcontroller only has 32
KB of flash memory and the LMIC library is taking most of it,
there is only very limited code space left for the application
dealing with the sensors (about 2 KB). Nevertheless, this is
sufficient to query some sensors like in our example the DHT22.

Now download and run the :ref:`Arduino_Sketch_Adafruit32u4.ino`
file in the Arduino IDE. This code was created by merging the example
code of both the sensors and the ttn-otaa example from the lmic library.
Some required changes were made while merging the example codes.
The user should change the network session key, app session
key and device address in the code before compiling. These keys
can be obtained from the TTN account as shown in the
`services section <#services>`_.

.. literalinclude:: Arduino_Sketch_Adafruit32u4/Arduino_Sketch_Adafruit32u4.ino
   :language: arduino
   :linenos:
   :caption: Modify the keys in highlighted lines.
   :name: Arduino_Sketch_Adafruit32u4.ino_Keys
   :lines: 143-155
   :emphasize-lines: 3,7,12

Following is the example code that can be used to measure the
battery voltage of the sensor node:

.. literalinclude:: Arduino_Sketch_Adafruit32u4/Arduino_Sketch_Adafruit32u4.ino
   :language: arduino
   :linenos:
   :caption: Code for measuring the battery voltage
   :name: Arduino_Sketch_Adafruit32u4.ino_BatteryVoltage
   :lines: 338-348

Services
--------

The services used for this sensor-node are:

- `TheThingsNetwork <#registration-of-the-sensor-node-with-the-things-network-ttn>`_ service for LoRaWAN network service.
- `TheThingsNetwork - OGC SensorWeb <#the-things-network-ogc-sensorweb-integration>`_ integration for uploading LoRaWAN sensor data into OGC infrastructure.

Registration of the sensor node with The Things Network (TTN)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The LoRaWAN protocol makes use of a number of different
identifiers, addresses, keys, etc. These are required to
unambiguously identify devices, applications, as well as to
encrypt and decrypt messages. The names and meanings are
`nicely explained on a dedicated TTN web page
<https://www.thethingsnetwork.org/docs/lorawan/addressing.html>`_.

The sketch given above connects the sensor node with The
Things Network (TTN) using the Activation-by-Personalisation
(ABP) mode. In this mode, the required keys for data encryption
and session management are created manually using the TTN
console window and must be pasted into the source code of the
sketch below. In order to get this running, you will need to
`create a new device in the TTN console window
<https://www.thethingsnetwork.org/docs/devices/registration.html>`_.
This assumes that you already have a TTN user account (which
needs to be created otherwise). In the settings menu of the
newly created device the ABP mode must be selected and the
settings must be saved. Then copy the DevAddr, the NwkSKey, and
the AppSKey from the TTN console web page of the newly
registered device and paste them into the proper places in the
sketch above. Please make sure that you choose for each of the
three keys the correct byte ordering (MSB for all three keys).
A detailed explanation of these steps is `given
here <https://learn.adafruit.com/the-things-network-for-feather?view=all>`_.
Then the sketch can be compiled and uploaded to the Adafruit
Feather 32u4 LoRa microcontroller.

**Important hint**: everytime the sensor node is reset or being
started again, make sure to reset the frame counter of the
registered sensor in the TTN console web page of the registered
device. The reason is that in LoRaWAN all transmitted data
packets have a frame counter, which is incremented after each
data frame being sent. This way a LoRaWAN application can avoid
receiving and using the same packet again (replay attack).
When TTN receives a data packet, it checks if the frame number
is higher than the last one received before. If not, the
received packet is considered to be old or a replay attack and
is discarded. When the sensor node is reset or being started
again, its frame counter is also reset to 0, hence, the TTN
application assumes that all new packages are old, because
their frame counter is lower than the last frame received
(before the reset). A manual frame counter reset is only
necessary when registering the node using ABP mode. In OTAA
mode the frame counter is automatically reset in the sensor
node and the TTN network server.

TTN Payload Decoding
^^^^^^^^^^^^^^^^^^^^

Everytime a data packet is received by a TTN application a
dedicated Javascript function is being called (Payload Decoder
Function). This function can be used to decode the received
byte string and to create proper Javascript objects or values
that can directly be read by humans when looking at the incoming
data packet. This is also useful to format the data in a
specific way that can then be forwarded to an external
application (e.g. a sensor data platform like `MyDevices
<https://mydevices.com/>`_ or `Thingspeak <https://thingspeak.com/>`_ ).
Such a forwarding can be configured in the TTN console in the
"Integrations" tab. :ref:`TTN_Payload_Decode` given here checks
if a packet was received on LoRaWAN port 7 and then assumes that
it consists of the 6 bytes encoded as described above.
It creates the three Javascript objects 'temperature', 'humidity',
and 'vbattery'. Each object has two fields: 'value' holds the
value and 'uom' gives the unit of measure. The source code can
simply be copied and pasted into the 'decoder' tab in the TTN
console after having selected the application. Choose the
option 'Custom' in the 'Payload Format' field. Note that when
you also want to handle other sensor nodes sending packets on
different LoRaWAN ports, then the Payload Decoder Function can
be extended after the end of the  if (port==7) {...} statement
by adding  else if (port==8) {...} else if (port==9) {...} etc.

The Things Network - OGC SensorWeb Integration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The presented Payload Decoder Function works also with the
`TTN-OGC SWE Integration <https://github.com/52North/ttn-ogcswe-integration>`_
for the `52° North Sensor Observation Service (SOS) <https://github.com/52North/SOS>`_.
This software component can be downloaded from `this repository <https://github.com/52North/ttn-ogcswe-integration>`_.
It connects a TTN application with a running transactional
`Sensor Observation Service 2.0.0 (SOS) <https://www.opengeospatial.org/standards/sos>`_.
Data packets received from TTN are imported into the SOS. The
SOS persistently stores sensor data from an arbitrary number
of sensor nodes and can be queried for the most recent as well
as for historic sensor data readings. The 52° North SOS comes
with its own REST API and a nice web client allowing to browse
the stored sensor data in a convenient way.

We are running an instance of the 52° North SOS and the TTN-OGC
SWE Integration. The web client for this LoRaWAN sensor node
can be accessed `on this page <http://129.187.38.201:8080/ttn-sos-integration/static/client/helgoland/index.html#/diagram?ts=ttnOGC__7,ttnOGC__8,ttnOGC__6>`_.
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

.. literalinclude:: Arduino_Sketch_Adafruit32u4/Arduino_Sketch_Adafruit32u4.ino
   :language: arduino
   :linenos:
   :caption: Arduino Sketch for Adafruit32u4 LoRa sensor node
   :name: Arduino_Sketch_Adafruit32u4.ino

.. literalinclude:: TTN_Payload_Decode.js
   :language: Javascript
   :linenos:
   :caption: TTN payload decoder for Adafruit32u4 LoRa sensor node
   :name: TTN_Payload_Decode

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
