#!/usr/bin/env python
import numpy as np
hprev = 0
filer = open("test.txt", "r")
filew = open("correct.txt", "a")
for line in filer:
    v, h, e, d = line.split(" ")
    v = float(v.replace("V",""))
    h = float(h.replace("H",""))
    d = float(d.replace("\n",""))
    #hnew = (h-405)%1000 #positive rotation correction
    #hnew = (h + (428-h)*2)%1000 #negative rotation correction
    filew.write("V" + str(v) + " " + "H" + str(hnew) + " = " + str(d) + "\n")
filer.close()
filew.close()
