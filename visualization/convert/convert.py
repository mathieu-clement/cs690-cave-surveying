#!/usr/bin/env python
import numpy as np
import sys

def spherical2cartesian(r, theta, phi):
    x = r * np.sin(theta*np.pi/180) * np.cos(phi*np.pi/180)
    z = r * np.sin(theta*np.pi/180) * np.sin(phi*np.pi/180)
    y = r * np.cos(theta*np.pi/180)
    return(x, y, z)

num_points = 0

with open(sys.argv[2], "w") as filew, open(sys.argv[1], "r") as filer:
    for line in filer:
        try:
            v, h, e, d = line.split(" ")
            v = float(v.replace("V",""))
            h = float(h.replace("H",""))
            d = float(d.replace("\n",""))
            (x, y, z) = spherical2cartesian(d, v-20, h/2.777777)
            filew.write("%f %f %f\n" %(x, y, z))
            num_points += 1
        except ValueError:
            continue


with open(sys.argv[2], "r+") as filew:
    content = filew.read()
    filew.seek(0, 0)
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
    filew.write(content)
