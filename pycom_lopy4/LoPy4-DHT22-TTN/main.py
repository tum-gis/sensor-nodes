from network import LoRa
import socket
import time
import ubinascii
import pycom
from machine import Pin
import dht22

while (True):
    pycom.heartbeat(False)
    pycom.rgbled(0x800000)

    # start a new measurement (taking 2 seconds) to ensure that the next 
    # retrieval of readings has current values
    dht.trigger()
    time.sleep(0.2)

    # now start a 2nd measurements which - according to the DHT22 datasheet -
    # delivers the measured values from the previous reading
    hasreading=False
    numtrials=0

    while(not hasreading):
        hasreading=dht.trigger()
        numtrials=numtrials+1
        if hasreading:
            print("RH = {}%  T = {}C".format(dht.humidity, dht.temperature))
        else:
            print(dht.status)

    hum_msb=int(dht.humidity*100/256)
    hum_lsb=int(dht.humidity*100%256)

    tmp_int=int(dht.temperature*100)
    # if temperature value is negative, then represent it by its 2's complement (16 bit)
    if (tmp_int<0):
        tmp_int=65536+tmp_int

    tmp_msb=int(tmp_int/256)
    tmp_lsb=int(tmp_int%256)
    print("RH = {} {}  T = {} {}".format(hum_msb, hum_lsb, tmp_msb, tmp_lsb))
    pycom.rgbled(0x000040)

    # make the socket blocking
    # (waits for the data to be sent and for the 2 receive windows to expire)
    s.setblocking(True)

    # send some data
    s.send(bytes([tmp_msb, tmp_lsb, hum_msb, hum_lsb, 0, 0]))

    # make the socket non-blocking
    # (because if there's no data received it will block forever...)
    s.setblocking(False)

    # get any data received (if any...)
    data = s.recv(64)
    print(data)

    pycom.heartbeat(True)
    # wait for such a time period that we have one measurement every 300 seconds
    time.sleep(300-numtrials*4-3)
