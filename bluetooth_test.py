import serial
import RPi.GPIO as GPIO
from time import sleep

bluez=serial.Serial("/dev/rfcomm0", baudrate=115200 )


case = None
while case == None:
    try:
        print "Please enter the case number: "
        case = raw_input(  )
        bluez.write(str(case))
        print bluez.readline()
        case = None
        
    except:
        pass # Ignore any errors that may occur and try again

