import time
import serial
ser = serial.Serial("/dev/ttyTHS1", baudrate = 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
counter=1


while 1:
    ser.flushInput()
    ser.flushOutput()
    ser.write(b"%d\n"%(counter))
    print("%d"%(counter))
    time.sleep(0.3)
    #counter += 1
ser.close()
