from __future__ import division
import usb
from usb.core import USBError
import time
import math
import matplotlib.pyplot as plt
import numpy as np


class ADF4158():
    def __init__(self):
        #Register definitions:
        # (Register: name: (First bit (LSB), Length))
        self.register_def = {
                0:{'ramp_on':(31, 1), 'muxout':(27, 4), 'n':(15, 12), 'frac_msb':(3, 12)},
                1:{'reserved0':(28, 4), 'frac_lsb':(15, 13), 'reserved1': (3, 12)},
                2:{'reserved2':(29, 3), 'csr_en':(28,1), 'cp_current':(24,4), 'reserved3':(23,1), 'prescaler':(22,1), 'rdiv2':(21,1), 'reference_doubler':(20,1), 'r_counter':(15,5), 'clk1_divider':(3,12)},
                3:{'reserved4':(16, 16), 'n_sel':(15,1), 'sd_reset':(14,1), 'reserved5':(12,2), 'ramp_mode':(10,2), 'psk_enable':(9,1), 'fsk_enable':(8,1), 'lpd':(7,1), 'pd_polarity':(6,1), 'power_down':(5,1), 'cp_3state':(4,1), 'counter_reset':(3,1)},
                4:{'lf_sel':(31,1), 'sd_mod_mode':(26,5), 'reserved6':(25,1), 'neg_bleed_current':(23,2), 'readback_to_muxout':(21,2), 'clk_div_mode':(19,2), 'clk2_divider':(7,12), 'reserved7':(3,4)},
                5:{'reserved8':(30,2), 'tx_ramp_clk':(29,1), 'par_ramp':(28,1), 'interrupt':(26,2), 'fsk_ramp_en':(25,1), 'ramp2_en':(24,1), 'dev_sel':(23,1), 'dev_offset':(19,4), 'deviation':(3,16)},
                6:{'reserved9':(24,8), 'step_sel':(23,1), 'step':(3,20)},
                7:{'reserved10':(19,13), 'ramp_del_fl':(18,1), 'ramp_del':(17,1), 'del_clk_sel':(16,1), 'del_start_en':(15,1), 'delay_start_divider':(3,12)}
        }
        self.registers = [0]*8
        self.modified = [False]*8

        #Check unique names
        keys = []
        for key in self.register_def.itervalues():
            for r in key:
                if r in keys:
                    raise Exception("Duplicate register {}".format(r))
                keys.append(r)

    def find_reg(self, reg):
        """Finds register by name"""
        for key, val in self.register_def.iteritems():
            if reg in val.keys():
                return key, val[reg]
        return None, None

    def write_value(self, **kw):
        """Write value to register, doesn't update the device"""
        for reg, val in kw.iteritems():
            #print "{} = {}".format(reg, val)
            reg_n, reg_def = self.find_reg(reg)
            if reg_n == None:
                raise ValueError("Register {} not found".format(reg))
            reg_start = reg_def[0]
            reg_len = reg_def[1]
            if val > 2**reg_len or val < 0:
                raise ValueError("Invalid value, got: {}, maximum {}".format(val, reg_len))
            #Clear previous value
            self.registers[reg_n] &= (~((((2**reg_len-1))&0xFFFFFFFF) << reg_start) & 0xFFFFFFFF)
            self.registers[reg_n] |= (val) << reg_start
            self.modified[reg_n] = True
        return

    def freq_to_regs(self, fout, fpd):
        """Output N, FRAC_MSB and FRAC_LSB register values to output fout given
        phase comparator frequency fpd"""
        n = int(fout/fpd)
        frac_msb = int(((fout/fpd) -n)*2**12)
        frac_lsb = int(((((fout/fpd) -n)*2**12)-frac_msb)*2**13)
        return n, frac_msb, frac_lsb

    def configure_ramp(self, f0, fpd, bw, length):
        n, frac_msb, frac_lsb = self.freq_to_regs(f0, fpd)

        #Configure start frequency
        self.write_value(n=n, frac_msb=frac_msb, frac_lsb=frac_lsb)

        clk1 = int((fpd*length)/2**20)+1

        print "clk1", clk1

        self.write_value(clk1_divider=clk1, clk2_divider=1)

        steps = int(fpd*length/clk1)

        print "steps",steps

        devmax = 2**15
        fres = fpd/2**25
        print "fres",fres

        fdev = bw/steps

        print "fdev",fdev

        dev_offset = int(math.ceil(math.log(fdev/(fres*devmax), 2)))
        dev_offset = max(0, dev_offset)
        print "dev offset",dev_offset

        fdev_res = fres * 2**dev_offset
        print "fdev_res", fdev_res

        dev = int(fdev/(fres*2**dev_offset))
        print "DEV",dev

        real_fdev = (fpd/2**25)*(dev*2**dev_offset)
        print "Real dev",real_fdev

        #Calculate the actual values from programmed ones
        real_length = steps*clk1/fpd
        real_bw = real_fdev*steps
        real_f0 = fpd*(n+( (frac_msb<<13)+frac_lsb)/2**25)

        print "Length {} ms, BW {} MHz, F0 {} MHz".format(1e3*real_length, 1e-6*real_bw, 1e-6*real_f0)

        self.write_value(deviation=dev, step=steps, dev_offset=dev_offset)
        self.write_value(clk_div_mode=3)
        self.write_value(ramp_on=1)
        #Length, BW, F0
        return (real_length, real_bw, real_f0)

    def to_device(self, device):
        """Writes the registers to the device"""
        self.write_adf4158(7, device, self.registers[7])

        self.write_value(step_sel=0)
        self.write_adf4158(6, device, self.registers[6])
        self.write_value(step_sel=1)
        self.write_adf4158(6, device, self.registers[6])

        self.write_value(dev_sel=0)
        self.write_adf4158(5, device, self.registers[5])
        self.write_value(dev_sel=1)
        self.write_adf4158(5, device, self.registers[5])

        self.write_adf4158(4, device, self.registers[4])
        self.write_adf4158(3, device, self.registers[3])
        self.write_adf4158(2, device, self.registers[2])
        self.write_adf4158(1, device, self.registers[1])
        self.write_adf4158(0, device, self.registers[0])

        self.modified = [False]*8

    def write_adf4158(self, control, device, word):
        if control > 7 or control < 0:
            raise ValueError("Invalid control")
        print "Write",control,word,(word >> 16) & 0x0000FFFF,(word & 0x0000FFF8) | control
        r = device.ctrl_transfer(0x40, 2, (word >> 16) & 0x0000FFFF, (word & 0x0000FFF8) | control)
        return r

    def read_adf4158(self, device):
        """Read from MUXOUT pin of the ADF4158, MUXOUT must have been configured
        previously"""
        word = device.ctrl_transfer(0xC0, 3, 0, 0, 4)
        return word


if __name__ == "__main__":
    if 1:
        device = usb.core.find(idVendor=0x1d50, idProduct=0x6099)

        if device == None:
            print "Device not found"
            exit(0)
        print "Found device"

        device.set_configuration()

        #for i in xrange(3):
        #    #Blink LED
        #    device.ctrl_transfer(0x40, 5, 0, 1<<4) #Disable LED
        #    time.sleep(0.1)
        #    device.ctrl_transfer(0x40, 4, 0, 1<<4) #Enable LED
        #    time.sleep(0.1)

        device.ctrl_transfer(0x40, 4, 0, 1<<3) #MIXER ENBL high
        device.ctrl_transfer(0x40, 4, 0, 1<<2) #PA on
        device.ctrl_transfer(0x40, 4, 0, 1<<1) #ADC on

        device.ctrl_transfer(0x40, 4, 0, 1) #ADF on

        device.ctrl_transfer(0x40, 6, 0, 0) # Set MCP gain
    else:
        device = None

    adf = ADF4158()
    # Set positive VCO polarity, 8/9 prescaler needed for high RF frequency
    # Disable R counter
    adf.write_value(pd_polarity=1, prescaler=1, r_counter=1)
    # Enable cycle slip reduction, Enable negative bleed current
    adf.write_value(csr_en=1, neg_bleed_current=3)
    adf.configure_ramp(5.4e9, 30e6, 300e6, 1000e-6)
    adf.write_value(ramp_mode=1) #Triangle
    adf.to_device(device)

    device.ctrl_transfer(0x40, 1, 1, 0) # Transceiver mode RX

    plt.ion()
    plt.show()
    pdata = [0]*10*512
    #line, = plt.plot(pdata)
    line = None
    plt.ylim([-1,1])
    i = 0
    tstart = time.time()
    try:
        while True:
                i = i+1
                data = device.read(0x81, 512)
                continue
                #print data
                s = []
                for d in data:
                    if d & (1<<7):
                        d = (~d + 1) & 0xff
                        d = -d
                    s.append(d/128.)
                pdata[i*512:(i+1)*512] = s
                print max(pdata)
                fft = 10*np.log(map(abs,np.fft.rfft(s)))
                #plt.ylim([-50,50])
                x = pdata
                if line == None:
                    line, = plt.plot(x)
                else:
                    line.set_ydata(x)  # update the data
                #line.set_ydata(pdata)  # update the data
                plt.draw()
                plt.pause(0.0001)
    except KeyboardInterrupt:
        tend = time.time()-tstart
        print tend, i*512, 1e-6*i*512/(tend)
    finally:
        #device.ctrl_transfer(0x40, 5, 0, 1<<2) #PA off
        #device.ctrl_transfer(0x40, 5, 0, 1<<1) #ADC off
        #device.ctrl_transfer(0x40, 5, 0, 1) #ADF off

        device.ctrl_transfer(0x40, 1, 0, 0) # Transceiver mode off

