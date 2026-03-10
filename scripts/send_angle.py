import serial
import time

# Change COM port if needed
ser = serial.Serial('COM3', 115200, timeout=1)

time.sleep(2)  # wait for ESP32 reset

while True:
    angle = input("Enter angle: ")

    ser.write((angle + "\n").encode())

    # Read response from ESP32
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        print(line)