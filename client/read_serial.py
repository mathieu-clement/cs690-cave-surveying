#!/usr/bin/env python3.4

import glob
import serial
import sys
import time

def arduino_is_ready(arduino_output):
    return arduino_output == b'Ready\r\n'

if not sys.platform.startswith('darwin'):
    sys.exit('Only MacOS is supported at this time. Program will now terminate.')

serial_ports = glob.glob('/dev/tty.usbmodem*')
print('Found USB modems:', serial_ports)
assert len(serial_ports) == 1, "More than one USB modem found."

print('Opening ', serial_ports[0])
ser = serial.Serial(serial_ports[0])
print('Using serial port', ser.name)

print('Waiting for "Ready" signal from Arduino')
possibly_ready_str = ser.readline()
assert arduino_is_ready(possibly_ready_str), 'Expected to receive Ready from Arduino but received ' + str(possibly_ready_str) + ' instead'

print('Sending "sweep" command')
ser.write(b'sweep')
ser.flush()

while ser.is_open:
    line = ser.readline()
    if arduino_is_ready(line):
        print('Received Ready signal again')
        break
    line = line.decode('utf-8')
    line = line.replace("\r\n","")
    print('Arduino says:', line)

print('Gracefully closing serial port')
ser.close()
