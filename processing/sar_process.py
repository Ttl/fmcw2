from __future__ import division

import numpy as np
import matplotlib.pyplot as plt
import cPickle as pickle
from scipy import interpolate as interp
from numpy.fft import fftshift, ifft, fft, ifft2
from scipy.stats import linregress

rs = 0
squint = 2
interpolate = 1
taylor_sl = 43
dynamic_range = 35
###

c = 299792458.0

def unwrap(a, fc):
    tol = (3e8/fc)/4
    offset = 0
    out = [a[0]]
    for i in xrange(1,len(a)):
        diff = a[i]-a[i-1]
        if diff > tol:
            offset += tol
        elif diff < -tol:
            offset -= tol
        out.append(a[i]+2*offset)
    return out

def entropy(im):
    intensity = np.abs(im)**2
    E = np.sum(intensity)
    with np.errstate(divide='ignore', invalid='ignore'):
        l = intensity*np.log2(intensity)
    return -np.sum(np.nan_to_num(l))/E+np.log2(E)

def ift(F, ax = -1):
    f = fftshift(ifft(fftshift(F), axis = ax))

    return f

def ft(F, ax = -1):
    f = fftshift(fft(fftshift(F), axis = ax))

    return f

def ift2(F, delta=1):
    N = F.shape[0]
    f = fftshift(ifft2(fftshift(F)))*(delta*N)**2

    return(f)

def taylor(nsamples, S_L=43):
    xi = np.linspace(-0.5, 0.5, nsamples)
    A = 1.0/np.pi*np.arccosh(10**(S_L*1.0/20))
    n_bar = int(2*A**2+0.5)+1
    sigma_p = n_bar/np.sqrt(A**2+(n_bar-0.5)**2)

    #Compute F_m
    m = np.arange(1,n_bar)
    n = np.arange(1,n_bar)
    F_m = np.zeros(n_bar-1)
    for i in m:
        num = 1
        den = 1
        for j in n:
            num = num*\
            (-1)**(i+1)*(1-i**2*1.0/sigma_p**2/(\
                            A**2+(j-0.5)**2))
            if i!=j:
                den = den*(1-i**2*1.0/j**2)

        F_m[i-1] = num/den

    w = np.ones(nsamples)
    for i in m:
        w += F_m[i-1]*np.cos(2*np.pi*i*xi)

    w = w/w.max()
    return(w)

def hilbert(x):
    """Hilbert transform. Generates complex IQ-signal from real signal."""
    fx = np.fft.fft(x)
    fx[:len(fx)/2] = 0
    fx[0] = 0 # Zero DC component
    return 2*np.fft.ifft(fx)

with open('sar_data.p', 'rb') as f:
    fc, bw, tsweep, data, range0, range1, delta_crange = pickle.load(f)

delta_crange *= 1
crange0 = -delta_crange*(len(data)-1)/2.
crange1 = delta_crange*(len(data)-1)/2.

#Hilbert transformation to get complex data
if type(data) == list or data[0].dtype != np.complex128:
    print "Hilbert transform"
    w = np.hamming(len(data[0]))
    data *= w
    data = np.array(map(hilbert, data))

#Insert here the phase error from autofocusing
errors = []

if len(errors) > 0:
    errors = np.array(errors)-min(errors)
    errors = (4*np.pi*fc/c)*errors
    errors = np.unwrap(errors)
    errors = errors/(4*np.pi*fc/c)
    n = np.arange(0,len(errors))
    #Remove linear term that moves the image in azimuth
    slope, intercept, _, _, _ = linregress(n,errors)
    line = slope*n+intercept
    errors -= line

    if 1:
        plt.plot(errors)
        plt.xlabel("Samples")
        plt.ylabel("Motion error [m]")

    if 1:
        t = np.linspace(0,1e-3,len(data[0]))
        for d in xrange(len(errors)):
            gamma = bw/tsweep
            e = -2*errors[d]/c
            data[d] *= np.exp(1j*(-2*np.pi*(fc+gamma*t)*e))

raw_extent = [range0, range1, crange0, crange1]

if 0:
    #Raw data
    shdata = [np.fft.fft(r) for r in data]
    plt.figure()
    plt.imshow(20*np.log10(np.abs(shdata)), aspect='auto', interpolation='none', extent=raw_extent)

if 0:
    plt.figure()
    plt.imshow(np.angle(data), aspect='auto', interpolation='none', extent=raw_extent)

print delta_crange

#Zeropad cross-range
if squint >= 1:
    zpad = squint*data.shape[0]
    zeros = np.zeros((zpad, data.shape[1]), dtype=np.complex)
    index = int(np.round((zpad - data.shape[0])/2))
    for i in xrange(data.shape[0]):
        zeros[index+i] = data[i]
    data = zeros

kx = np.linspace(-np.pi/delta_crange, np.pi/delta_crange, len(data))
kr = np.linspace(((4*np.pi/c)*(fc)), ((4*np.pi/c)*(fc+bw)), len(data[0]))

#Along the track FFT
cfft = ift(data, ax = 0)

if 0:
    plt.figure()
    plt.imshow(np.angle(cfft), aspect='auto', extent=[kr[0], kr[-1], kx[0], kx[-1]])
    plt.figure()
    plt.imshow(np.abs(cfft), aspect='auto', extent=[kr[0], kr[-1], kx[0], kx[-1]])

#Matched filter
if rs != 0:
    phi_mf = np.zeros(cfft.shape)
    for ii in xrange(cfft.shape[1]):
        for jj in xrange(cfft.shape[0]):
            phi_mf = rs*(kr[ii]**2-kx[jj]**2 )**0.5

    smf = np.exp(1j*phi_mf)
    cfft = cfft*smf

ky0 = (kr[0]**2 - kx[0]**2 )**0.5
if np.isnan(ky0):
    raise Exception("Ky0 = NaN")

points = (kr[-1]-ky0)/(kr[1]-kr[0])
ky_even = np.linspace(ky0, kr[-1], points)
range_scale = points/cfft.shape[1]

st = np.zeros((cfft.shape[0], points), dtype=np.complex)

#Stolt interpolation
for i in xrange(len(kx)):
    ky = (kr**2 - kx[i]**2 )**0.5
    ci = interp.interp1d(ky, cfft[i], fill_value=0, bounds_error=False)
    st[i,:] = (ci(ky_even))

if 0:
    plt.figure()
    plt.imshow(np.angle(st), aspect='auto', interpolation='none', extent=[ky_even[0], ky_even[-1], kx[0], kx[-1]])

if 0:
    d = np.fft.ifft(st, axis=0)
    d = np.fft.ifft(d, axis=1)
    plt.figure()
    plt.imshow(np.abs(d), aspect='auto', interpolation='none', extent=[kr[0], kr[-1], kx[0], kx[-1]])

#Create window
win_x = taylor(st.shape[1],taylor_sl)
win_x = np.tile(win_x, [st.shape[0],1])

win_y = taylor(st.shape[0],taylor_sl)
win_y = np.array([win_y]).T
win_y = np.tile(win_y, [1,st.shape[1]])

#win = win_x*win_y
win = win_y

#Apply window
st *= win

#Pad Spectrum
if 0:
    length_x = 2**(int(np.log2(st.shape[1]*interpolate))+1)
    length_y = 2**(int(np.log2(st.shape[0]*interpolate))+1)
    pad_x = length_x-st.shape[1]
    pad_y = length_y-st.shape[0]
    st = np.pad(st,((pad_y/2, pad_y/2),(pad_x/2, pad_x/2)), mode = 'constant')

#IFFT
st = ift2(st)
st = fftshift(st, 1)
st = st[:,:int(st.shape[1]/range_scale)]

crange0 *= squint
crange1 *= squint

plt.figure()

print entropy(st)
st = 20*np.log10(np.abs(st))
#, cmap=plt.cm.Greys_r
imgplot = plt.imshow(st, interpolation='none', extent=[range0, range1, crange0, crange1])
plt.xlabel("Range [m]")
plt.ylabel("Cross-range [m]")
m = np.max(st)
#Limit the dynamic range to clean the rounding errors
imgplot.set_clim(m-dynamic_range,m)
plt.show()
