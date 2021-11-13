import serial
from bluetooth import *
nearby_devices = discover_devices(lookup_names = True)
print(nearby_devices)
