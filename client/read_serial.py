#!/usr/bin/env python3.6

import glob
import logging
import serial
import sys
import time

logging.basicConfig()
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

def arduino_is_ready(arduino_output):
    return arduino_output == b'Ready\r\n'

if not sys.platform.startswith('darwin'):
    sys.exit('Only MacOS is supported at this time. Program will now terminate.')

baudrate=115200

if len(sys.argv) == 3:
    ser = serial.Serial(sys.argv[2],baudrate=baudrate)
else:
    serial_ports = glob.glob('/dev/tty.usbmodem*')
    logger.info('Found USB modems: %s', serial_ports)
    assert len(serial_ports) == 1, "More than one USB modem found."

    logger.info('Opening %s', serial_ports[0])
    ser = serial.Serial(serial_ports[0], baudrate=baudrate)

logger.info('Using serial port %s', ser.name)

logger.info('Waiting for "Ready" signal from Arduino')
possibly_ready_str = ser.readline()
assert arduino_is_ready(possibly_ready_str), 'Expected to receive Ready from Arduino but received ' + str(possibly_ready_str) + ' instead'

logger.info('Sending "sweep" command')
ser.write(b'sweep')
ser.flush()

last_vertical = ''
minV = 30
maxV = 140
rangeV = maxV - minV

with open(sys.argv[1], 'w') as f:
    while ser.is_open:
        line = ser.readline()
        if arduino_is_ready(line):
            logger.info('Received Ready signal again')
            break
        line = line.decode('utf-8')
        line = line.replace("\r\n","")
        parts = line.split(" ")
        vertical = parts[0]
        if vertical[0] == 'V' and vertical != last_vertical:
            v = int(vertical[1:])
            percents = round(100 * ((rangeV - v + minV) / rangeV))
            logger.info('%s (%d %%)', vertical, percents)
            last_vertical = vertical
        f.write(line)
        f.write('\n')

logger.info('Gracefully closing serial port')
ser.close()
