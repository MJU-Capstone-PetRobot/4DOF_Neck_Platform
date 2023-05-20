import serial


py_serial = serial.Serial(
    port = 'COM4',
    baudrate = 115200,
)

#파이썬에 입력
while 1:
    theta1 = input('rx ='); xn = float(theta1) #roll
    theta2 = input('ry ='); yn = float(theta2) #pitch
    theta3 = input('z ='); zn = float(theta3)  #z-axis 
    z = input('rz ='); zn = float(z)  #yaw 축(개발중)

    while 1:

        motorinput = str(round(theta1))+ ","+ str(round(theta2))+ ","+ str(round(theta3))+ ","+ str(round(z)) +"\n"
        print(motorinput)
        py_serial.write(motorinput.encode())
  

    
