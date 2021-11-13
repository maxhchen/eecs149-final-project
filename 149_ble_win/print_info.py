from bluetooth import *

from pprint import pprint
devices = discover_devices()
pprint(devices)