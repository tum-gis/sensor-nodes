from network import LoRa
import socket
import time
import ubinascii
import pycom
from machine import Pin
import dht22

# Initialise LoRa in LORAWAN mode.
# Please pick the region that matches where you are using the device:
# Asia = LoRa.AS923
# Australia = LoRa.AU915
# Europe = LoRa.EU868
# United States = LoRa.US915
lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868)

# Create the OTAA authentication parameters:
# directly copy the values from the Things Network Console and replace the
# xxxx's and yyyy's by these values (do not prepend anything like '0x' or similar)
app_eui = ubinascii.unhexlify('xxxxxxxxxxxxxxxx')
app_key = ubinascii.unhexlify('yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy')

print("Initializing DHT22 Sensor... ",end='')
dht = dht22.device(Pin.exp_board.G22)
print("ready!\n")

pycom.heartbeat(False)
pycom.rgbled(0xFF0000)

# join a network using OTAA (Over the Air Activation)
lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0)

# wait until the module has joined the network
while not lora.has_joined():
    time.sleep(2.5)
    print('Not yet joined...')

pycom.rgbled(0x00FF00)
time.sleep(1)

# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

# set the LoRaWAN port number for transmitted packets
s.bind(7)

pycom.heartbeat(True)
