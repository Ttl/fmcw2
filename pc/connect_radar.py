import radars
import usb.core

def get_class(r):
    """Return a radar class by name"""
    c = r
    c = r.capitalize()
    return getattr(getattr(radars,r), c)

def list_radars():
    """List radars imported in radars/__init__.py"""
    d = dir(radars)
    #Remove python builtins
    d = [i for i in d if i[:2] != '__']
    return d

def find_radar():
    found = None
    for r in list_radars():
        #Remove dummy radars
        if hasattr(r, 'dummy'):
            continue
        radar = get_class(r)
        usb_id = radar.usb_id
        dev = usb.core.find(idVendor=usb_id[0], idProduct=usb_id[1])
        if dev is None:
            continue
        found = radar
        break

    return found
