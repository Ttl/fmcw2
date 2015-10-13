from __future__ import division

import numpy as np
import matplotlib.pyplot as plt
import cPickle as pickle
from scipy import interpolate as interp
from numpy.fft import fftshift, fft, ifft, ifft2
from scipy.optimize import curve_fit
import time

rs = 0
squint = 2
interpolate = 1
taylor_sl = 30
dynamic_range = 30

###

c = 299792458.0

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
    fc, bw, tsweep, raw_data, range0, range1, delta_crange = pickle.load(f)

crange0 = -delta_crange*(len(raw_data)-1)/2.
crange1 = delta_crange*(len(raw_data)-1)/2.

#Hilbert transformation to get complex data
if type(raw_data) == list or raw_data[0].dtype != np.complex128:
    print "Hilbert transform"
    w = np.hamming(len(raw_data[0]))
    raw_data *= w
    raw_data = np.array(map(hilbert, raw_data))

def sar_entropy(data, fc, bw, tsweep, delta_crange, errors, window=True):
    focus = np.ones(data.shape, dtype=np.complex)
    
    if 1:
        #t = np.linspace(0,tsweep,len(data[0]))
        #gamma = bw/tsweep
        for e in xrange(len(errors)):
            if e==0:
                continue
            p = -2*errors[e]/c
            focus[e] = np.exp(1j*(-2*np.pi*(fc)*p))
            #focus[e] = np.exp(1j*(-2*np.pi*(fc+gamma*t)*p))
    
    st = data*focus
    #Zeropad cross-range
    if squint >= 1:
        zpad = squint*st.shape[0]
        zeros = np.zeros((zpad, st.shape[1]), dtype=np.complex)
        index = int(np.round((zpad - st.shape[0])/2))
        for i in xrange(st.shape[0]):
            zeros[index+i] = st[i]
        st = zeros
    
    kx = np.linspace(-np.pi/delta_crange, np.pi/delta_crange, len(st))
    kr = np.linspace(((4*np.pi/c)*(fc)), ((4*np.pi/c)*(fc+bw)), len(st[0]))
    
    #Along the track FFT
    st = ift(st, ax = 0)

    ky0 = (kr[0]**2 - kx[0]**2 )**0.5
    if np.isnan(ky0):
        raise Exception("Ky0 = NaN")
        
    ky_even = np.linspace(ky0, kr[-1], st.shape[1])
    
    #Stolt interpolation
    for i in xrange(len(kx)):
        ky = np.sqrt(kr**2 - kx[i]**2 )
        ci = interp.interp1d(ky, st[i,:], fill_value=0, bounds_error=False)
        st[i,:] = (ci(ky_even))
    
    if window:
        #Create window
        win_x = taylor(st.shape[1],taylor_sl)
        win_x = np.tile(win_x, [st.shape[0],1])
        
        win_y = taylor(st.shape[0],taylor_sl)
        win_y = np.array([win_y]).T
        win_y = np.tile(win_y, [1,st.shape[1]])
        
        win = win_x*win_y
        
        #Apply window
        st *= win
    
    #Pad Spectrum
    if 0:
        length_x = 2**(int(np.log2(st.shape[1]*interpolate))+1)
        length_y = 2**(int(np.log2(st.shape[0]*interpolate))+1)
        pad_x = length_x-st.shape[1]
        pad_y = length_y-st.shape[0]
        st = np.pad(st,((pad_y/2, pad_y/2),(pad_x/2,pad_x/2)), mode = 'constant')
  
    #IFFT
    st = ift2(st)
    st = fftshift(st, 1)

    return st

def surrogate(x, w, a, p, c):
    return a*np.cos(x+p)+c

def surrogate_min(raw_data, fc, bw, tsweep, delta_crange, iterations=20):
    wl = 3e8/fc
    surrogate = lambda x,a,p,c : a*np.cos(4*np.pi*x/wl+p)+c
    errors = [0]*len(raw_data)
    #Base for curve fitting
    es = np.linspace(0,wl/4,3)

    try:
        for i in xrange(iterations):
            print "Iteration",i
            for k in xrange(len(errors)):
                n = []
                for e in es:
                    errors[k] = e
                    n.append(entropy(sar_entropy(raw_data, fc, bw, tsweep, delta_crange, errors, False)))
                popt, pcov = curve_fit(surrogate, es, n)
                d = -wl*(popt[1]-np.pi)/(4*np.pi)
                # Check that point is minimum from second derivative
                if -popt[0]*np.cos(popt[1]+4*np.pi*d/wl) < 0:
                    d = -wl*(popt[1])/(4*np.pi)
                p = surrogate(d, *popt)
                #Wrap phase
                if 1:
                    s = np.sign(d)
                    d = abs(d)
                    while abs(d) > wl/2:
                        d -= wl/2
                    d *= s
                assert p - surrogate(d, *popt) < 1e-5
                errors[k] = d
            print "Entropy",p
            print errors
    except KeyboardInterrupt:
        print "Aborting"
    return errors


t_start = time.time()
m = surrogate_min(raw_data, fc, bw, tsweep, delta_crange)

print m
print "Done in", time.time() - t_start, "s"
#plt.figure()
#plt.plot(m)

st = sar_entropy(raw_data, fc, bw, tsweep, delta_crange, m)
print entropy(st)
crange0 *= squint
crange1 *= squint

plt.figure()
st = 20*np.log10(np.abs(st))
imgplot = plt.imshow(st, aspect='auto', interpolation='none', extent=[range0, range1, crange0, crange1])
db_max = np.max(st)
#Limit the dynamic range to clean the rounding errors
imgplot.set_clim(db_max-dynamic_range,db_max)
plt.show()
