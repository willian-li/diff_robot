import serial
import sys

def open_serial():

    ser = serial.Serial('/dev/ttyS3', 9600)  # 串口和波特率
    print("Opening Serial")
    

    if ser.in_waiting > 0:
            bufferRTT = list(ser.read(32))
            print(bufferRTT)
      
        



if __name__ == "__main__":
    device_port = "/dev/ttyS3"
    baudrate = 9600
    
    open_serial()
    
   