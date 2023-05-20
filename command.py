import serial


py_serial = serial.Serial(
    port = 'COM4',
    baudrate = 115200,
)

#파이썬에 입력
while 1:
    theta1 = input('rx ='); xn = float(theta1)
    theta2 = input('ry ='); yn = float(theta2)
    theta3 = input('rz ='); zn = float(theta3)
    z = input('rz ='); zn = float(z)

    while 1:

        motorinput = str(round(theta1))+ ","+ str(round(theta2))+ ","+ str(round(theta3))+ ","+ str(round(z)) +"\n"
        print(motorinput)
        py_serial.write(motorinput.encode())
  

    
