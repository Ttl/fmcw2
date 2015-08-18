import usb
import struct

device = usb.core.find(idVendor=0x1d50, idProduct=0x6099)

if device == None:
    print "Device not found"
    exit(0)
print "Found device"

device.set_configuration()

def board_id(device):
    return device.ctrl_transfer(0xC0, 14, 0, 0, 1)[0]

def version_string(device):
    return ''.join(map(chr,device.ctrl_transfer(0xC0, 15, 0, 0, 64)))

def part_id_serial(device):
    part_id = device.ctrl_transfer(0xC0, 18, 0, 0, 64)
    unpacked = []
    for i in xrange(6):
        unpacked.append(hex(struct.unpack_from('<L',part_id[4*i:])[0]))
    return unpacked

print "Firmware version:",version_string(device)
print "Board ID:", board_id(device)
serial = part_id_serial(device)
print "Part ID {} {}".format(*serial[:2])
print "Serial Number {} {} {} {}".format(*serial[2:])
