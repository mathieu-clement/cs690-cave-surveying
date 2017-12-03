./shifter.py sutro3.txt sutro3_shift1.txt 60 93 440
./shifter.py sutro3_shift1.txt sutro3_shift2.txt 60 78 30
./shifter.py sutro3_shift2.txt sutro3_shift3.txt 30 78 730
../convert/convert.py sutro3_shift3.{txt,pcd} 
