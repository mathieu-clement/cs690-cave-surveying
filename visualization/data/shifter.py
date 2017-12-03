#!/usr/bin/env python

import sys

input_filename = sys.argv[1]
output_filename = sys.argv[2]

min_v = int(sys.argv[3]) # incl.
max_v = int(sys.argv[4]) # incl.
shift_h = int(sys.argv[5])

with open(input_filename, 'r') as filer, open(output_filename, 'w') as filew:
    for line in filer:
        parts = line.split(" ")
        v = int(parts[0][1:])
        h = int(parts[1][1:])
        d = int(parts[3][:-1])

        if v >= min_v  and v <= max_v:
            h = h + shift_h
            if h < 0:
                h = 1000 + h
            if h > 1000:
                h = h - 1000
        filew.write('V%d H%d = %d\n' % (v, h, d))
