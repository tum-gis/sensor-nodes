
# dht22.py
#
# Class file for accessing the DHT22 temperature and humidity sensor using a WiPy 2.0.
#
# 2018 - Erik de Lange

from machine import Pin

import pycom
import time

class device:

    def __init__(self, pin):
        self.temperature = None
        self.humidity = None
        self.status = "NoConversionStartedError"
        self.pin = Pin(pin, mode=Pin.OPEN_DRAIN)

    def trigger(self):
        self.pin(1)
        time.sleep(2)  # enforce two second read interval

        self.pin(0)  # send start signal (1ms low).
        time.sleep_ms(1)

        pulses = pycom.pulses_get(self.pin, 100)  # capture communication

        self.pin.init(Pin.OPEN_DRAIN)

        if len(pulses) != 82:  # 40 data bit plus one acknowledge expected
            self.status = "ReadError - received {} only pulses".format(len(pulses))
            return False

        bits = []

        for level, duration in pulses[1:]:
            if level == 1:
                bits.append(0 if duration < 50 else 1)  # convert to 0 or 1

        data = []

        for n in range(5):
            byte = 0
            for i in range(8):  # shift 8 bits into a byte
                byte <<= 1
                byte += bits[n * 8 + i]
            data.append(byte)

        int_rh, dec_rh, int_t, dec_t, csum = data

        if ((int_rh + dec_rh + int_t + dec_t) & 0xFF) != csum:
            self.status = "Checksum Error"
            return False

        self.humidity = ((int_rh * 256) + dec_rh) / 10
        self.temperature = (((int_t & 0x7F) * 256) + dec_t) / 10
        if (int_t & 0x80) > 0:
            self.temperature *= -1

        self.status = "OK"
        return True


if __name__ == "__main__":
    dht = device(Pin.exp_board.G22)

    for _ in range(5):
        if dht.trigger() == True:
            print("RH = {}%  T = {}C".format(dht.humidity, dht.temperature))
        else:
            print(dht.status)
