import serial
import math


py_serial = serial.Serial(
    port = 'COM6',
    baudrate = 115200,
)

#파이썬에 입력
while 1:
    theta1 = input('rx ='); xn = float(theta1) #roll
    theta2 = input('ry ='); yn = float(theta2) #pitch
    theta3 = input('z ='); zn = float(theta3)  #z-axis 
    yaw = input('rz ='); yawn = float(yaw)  #yaw 축(개발중)

    motorinput = str(round(xn))+ ","+ str(round(yn))+ ","+ str(round(zn))+ ","+ str(round(yawn)) +"\n"
    print(motorinput)
    py_serial.write(motorinput.encode())


    
