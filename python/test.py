#!/usr/bin/env python3
import sys                           
import serial.tools.list_ports
import ui2c
from ui2c import UartI2C

is_windows = hasattr(sys, 'getwindowsversion')
bus_name = "ui2c:/dev/ttyUSB0:115200"    # or ui2c:COM12:115200 for windows
if(is_windows):
    bus_name = "ui2c:COM12:115200"       #

def find_available_serial_port():
    port_objs = serial.tools.list_ports.comports()
    unsorted_ports = []
    for port in port_objs:
        if(is_windows):
            if port.device.startswith('COM'):
                # remember and continue, we are looking for the last one
                unsorted_ports.append(port.device)
                #print(port.device)
        else:
            if port.device.startswith('/dev/ttyUSB') or port.device.startswith('/dev/ttyACM'):
                unsorted_ports.append(port.device)
                #return port.device
        #end if()
    #end for()
    
    offs = 3 if is_windows else 11
    ports = sorted(unsorted_ports, key=lambda x: -1*int(x[offs:]))

    for port_name in ports:
        #print(port_name)
        if(ui2c.probe_ui2c_device(port_name)):
            return port_name
    # end for()

    return None
#end find_available_serial_port()

def main():

    if '-b' in sys.argv:
        index = sys.argv.index('-b')
        if index + 1 < len(sys.argv):
            bus_name = sys.argv[index + 1]
    else:
        available_serial_port = find_available_serial_port()
        if(available_serial_port == None):
            print("No suitable ports found")
            return;
        bus_name = "ui2c:"+available_serial_port+":115200"
        print("Using "+bus_name)

    # Scan bus
    bus = UartI2C(available_serial_port)
    for i2c_addr in range(1, 119):
        if(bus.i2c_probe_dev(i2c_addr)):
            print("Found device @addr "+hex(i2c_addr))
    # end for()

    return
#end main()

if __name__ == '__main__':
    main()
