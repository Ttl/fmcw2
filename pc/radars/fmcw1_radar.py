import numpy as np
import time
import serial
import serial.tools.list_ports

class Fmcw1_radar():
    usb_id = (0x2504, 0x0300)

    def __init__(self):
        self.name = "FMCW1 Radar"
        self.serial = None

        self.rf_power = False
        self.fstep = 0
        self.fstart = 5.5e9
        self.fstop = 6.0e9
        self.bb_gain = 128
        self.trig = None
        self.debug_enable = True

        self.bb_srate = 10e6
        self.sweep_length = 0.5e-3

    def connect(self):
        ports = serial.tools.list_ports.comports()
        for dev, name, desc in ports:
            if desc == 'USB VID:PID=2504:0300':
                try:
                    self.serial = serial.Serial(
                        port=dev,
                        baudrate=1e6,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_TWO,
                        bytesize=serial.EIGHTBITS,
                        timeout=0.1
                    )
                    if not self.serial.isOpen():
                        raise Exception
                except:
                    print 'Opening serial port {} failed.'.format(dev)
                    raise
        if self.serial:
            #Set FMCW mode
            self.serial.write('F')
            return True
        return False

    def disconnect(self):
        if self.serial != None:
            self.serial.write('m')
            self.serial.close()

    def debug_print(self, s):
        if self.debug_enable:
            print s

    def set_rfpower(self, on):
        self.debug_print("set_rfpower {}".format(on))
        if on:
            self.serial.write('O')
        else:
            self.serial.write('o')

    def set_trig(self, trig):
        self.debug_print("set_trig {}".format(trig))
        self.trig = trig

    def set_sweep(self, start, stop, length):
        self.fstart = float(start)
        self.fstop = float(stop)
        self.sweep_length = float(length)
        self.debug_print("set_sweep {:.4} {:.4} {:.4}".format(start, stop, length))

    def stream(self):
        #Start streaming samples
        self.serial.write('m')
        while True:
            i = 0
            while self.serial.inWaiting() == 0 and i < 500:
                time.sleep(0.01)
                i += 1
            if i >= 500:
                raise Exception("Measure failed")
            d = []
            while True:
                c = self.serial.read(1)
                if c == 'M':
                    break
            #Gain
            c_mcp = ord(self.serial.read(1))

            #Number of points
            c = self.serial.read(2)
            points = (ord(c[0])<<8)|ord(c[1])
            overflows = 0
            abs_max = 0
            prev_c = 0
            while points > 0:
                if 1:
                    if len(d) == 0:
                        c = self.serial.read(2)
                        uint = (ord(c[0])<<8)|ord(c[1])
                    else:
                        c = self.serial.read(1)
                        #Reassemble int
                        uint = ord(c)
                        #Overflow, reading 16 bits
                        if uint == 128:
                            overflows +=1
                            c = self.serial.read(2)
                            uint = (ord(c[0])<<8)|ord(c[1])
                        else:
                            if uint > 127:
                                uint = -(256-uint)
                            uint *= 2
                            uint += prev_c
                    prev_c = uint
                    v = abs(uint-32768)
                    if v > abs_max:
                        abs_max = v
                    d.append(uint/float(2**16)-0.5)
                    points -= 1
                else:
                    c = self.serial.read(2)
                    #Reassemble int
                    uint = (ord(c[0])<<8)|ord(c[1])
                    #d.append(uint)
                    d.append(uint/float(2**16)-0.5)
                    points -= 1
            yield d

