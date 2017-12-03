#!/usr/bin/env python
import numpy as np
import sys

def spherical2cartesian(r, theta, phi):
    x = r * np.sin(theta*np.pi/180) * np.cos(phi*np.pi/180)
    z = r * np.sin(theta*np.pi/180) * np.sin(phi*np.pi/180)
    y = r * np.cos(theta*np.pi/180)
    return(x, y, z)

class Coordinate:
    def __init__(self, v, h):
        self.v = int(v)
        self.h = int(h)

    def __hash__(self):
        return hash((self.v, self.h))

    def __eq__(self, other):
        return self.v == other.v and self.h == other.h

points = {}

with open(sys.argv[1], "r") as filer:
    for line in filer:
        try:
            v, h, e, d = line.split(" ")
            v = float(v[1:])
            h = float(h[1:])
            d = float(d[:-1])
            if d < 10:
                continue
            
            coord = Coordinate(v, h)
            
            if coord not in points:
                points[coord] = []
            points[coord].append(d)
        except ValueError:
            continue

def write_cartesian(filew, v, h, d):
    (x, y, z) = spherical2cartesian(d, v-20, h/2.777777)
    filew.write("%f %f %f\n" % (x, y, z))

with open(sys.argv[2], "w") as filew:
    num_points = len(points)

    filew.write("# .PCD v.7 - Point Cloud Data file format\n")
    filew.write("VERSION .7\n")
    filew.write("FIELDS x y z\n")
    filew.write("SIZE 4 4 4\n")
    filew.write("TYPE F F F\n")
    filew.write("COUNT 1 1 1\n")
    filew.write("WIDTH %d\n" % num_points)
    filew.write("HEIGHT 1\n")
    filew.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    filew.write("POINTS %d\n" % num_points)
    filew.write("DATA ascii\n")
    
    for coord, distances in points.items():
        d = distances[0] if len(distances) == 1 else sum(distances)/len(distances)
        write_cartesian(filew, coord.v, coord.h, d)
