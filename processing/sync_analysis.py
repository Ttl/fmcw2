import sys
import matplotlib.pyplot as plt
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import struct

def most_common(lst):
    return max(set(lst), key=lst.count)

with open(sys.argv[1], 'r') as f:
    samples_in = f.read()
    syncs = []
    for i in xrange(0,len(samples_in),4):
        d = samples_in[i:i+4]
        syncs.append(struct.unpack('<L',d)[0])

#First find the minimum period
#Filter for too short periods
min_sync = most_common(syncs[:100])
print "Most common sync", min_sync

if 0:
    fixed_syncs = []
    for s in syncs:
        if float(s)/min_sync > 1.5:
            a = [min_sync]*((s-min_sync)/min_sync)
            a.append(s-(min_sync)*((s/min_sync)-1))
            assert sum(a) == s
            fixed_syncs.extend(a)

    syncs = fixed_syncs

short_syncs = 0
long_syncs = 0
for e,sync in enumerate(syncs):
    if sync < min_sync - 10:
        print e,sync
        short_syncs += 1
    if sync > min_sync + 10:
        print e,sync
        long_syncs += 1

print "{} syncs".format(len(syncs))
print "{} short syncs".format(short_syncs)
print "{} long syncs".format(long_syncs)
