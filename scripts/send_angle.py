import serial
import time

ser = serial.Serial('COM3',115200)
time.sleep(2)

while True:

    angle1 = input("Enter Motor 1 Angle: ")
    angle2 = input("Enter Motor 2 Angle: ")

    command = f"{angle1},{angle2}\n"

    ser.write(command.encode())
    ser.flush()

    print("Sent:", command.strip())