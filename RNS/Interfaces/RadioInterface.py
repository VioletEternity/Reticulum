# MIT License
#
# Copyright (c) 2023 Maya <maya@maybemaya.me>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import threading
import struct
import socket
import select
import os
import RNS
from .Interface import Interface


class LoopbackRadio:
    def __init__(self):
        self.r, self.w = os.pipe()

    @property
    def pollfd(self):
        return self.r
    
    def recv(self):
        return [os.read(self.r, 512)]

    def send(self, data):
        os.write(self.w, data)


class SX127xRadio:
    def __init__(self, phy_type, spi_path, gpio_path, rst_pin, dio0_pin, frequency, 
                 bandwidth, spreading_factor, coding_rate, tx_power):
        self.phy = phy_type(
            spipath = spi_path,
            gpiopath = gpio_path,
            rstpin = int(rst_pin),
            dio0pin = int(dio0_pin),
        )
        self.phy.frequency     = int(frequency)
        self.phy.bandwidth     = int(bandwidth)
        self.phy.spread_factor = int(spreading_factor)
        self.phy.coding_rate   = int(coding_rate)
        self.phy.txpower       = int(tx_power)

        self._rx_queue = []
        self.phy._on_receive = self._rx_queue.append

        self.phy.receive_continuous()
    
    @property
    def pollfd(self):
        return self.phy.pollfd

    def recv(self):
        self.phy.poll(timeout=0)

        result = self._rx_queue[:]
        self._rx_queue.clear()

        result = [data[1:] for data in result] # FIXME: remove header
        return result

    def send(self, data):
        data = b"\xf0" + data # FIXME: prepend stub header

        self.phy.idle()

        self.phy.begin_packet()
        self.phy.write(data)
        self.phy.end_packet()

        self.phy.receive_continuous()


class RadioInterface(Interface):
    def __init__(self, owner, name, radio_type, params):
        self.rxb = 0
        self.txb = 0

        self.owner      = owner
        self.name       = name
        self.radio_type = radio_type
        self.params     = params

        try:
            self.configure_device()
        except Exception as e:
            RNS.log("Could not open radio interface "+str(self), RNS.LOG_ERROR)
            raise e

    def configure_device(self):
        RNS.log("Configuring radio interface for "+self.radio_type+" with "+repr(self.params)+"...", RNS.LOG_VERBOSE)

        if self.radio_type == "LoopbackRadio":
            self.radio = LoopbackRadio(**self.params)
        elif self.radio_type == "SX1276Radio":
            from sx127x import sx1276
            self.radio = SX127xRadio(sx1276, **self.params)
        elif self.radio_type == "SX1278Radio":
            from sx127x import sx1278
            self.radio = SX127xRadio(sx1278, **self.params)
        else:
            RNS.log("Unknown radio interface type "+self.radio_type, RNS.LOG_CRITICAL)
            RNS.panic()
        
        self.pipe_worker, self.pipe_application = os.pipe()
        thread = threading.Thread(target=self.radioWorker)
        thread.daemon = True
        thread.start()
    
    @property
    def bitrate(self):
        return 1_000_000

    def radioWorker(self):
        while True:
            rready, _, _ = select.select([self.radio.pollfd, self.pipe_worker], [], [])
            if self.radio.pollfd in rready:
                for data in self.radio.recv():
                    self.processIncoming(data)
            if self.pipe_worker in rready:
                length, = struct.unpack("I", os.read(self.pipe_worker, struct.calcsize("I")))
                self.radio.send(os.read(self.pipe_worker, length))

    def processIncoming(self, data):
        # print("processIncoming", data.hex())
        self.rxb += len(data)            
        self.owner.inbound(data, self)

    def processOutgoing(self, data):
        # print("processOutgoing", data.hex())
        os.write(self.pipe_application, struct.pack("I", len(data)) + data)
        self.txb += len(data)

    def __str__(self):
        return "RadioInterface."+self.radio_type+"["+self.name+"]"
