
import RPi.GPIO as GPIO
import smbus
import time

# Configure GPIO
GPIO.setmode(GPIO.BCM)

# Pins For I2C Mux Address 
MUX_0 = 25
MUX_1 = 24
MUX_2 = 23

# Configure GPIO Pins To Output
GPIO.setup(MUX_0, GPIO.OUT)
GPIO.setup(MUX_1, GPIO.OUT)
GPIO.setup(MUX_2, GPIO.OUT)

# Configure address
GPIO.output(MUX_0, 0)
GPIO.output(MUX_1, 0)
GPIO.output(MUX_2, 0)

MUX_ADDR = 0x70
M_1 = 0x00
DEV_ADDR = 0x50

BUS = smbus.SMBus(1)

print("------------------------------------")
try:
	print("Checking addr 1")
	a = BUS.write_byte(MUX_ADDR, 0b00000001)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x00)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x01)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x02)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x03)
	print(a)
except:
	print("No device")
	
print("------------------------------------")
try:
	print("Checking addr 2")
	a = BUS.write_byte(MUX_ADDR, 0b00000010)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x00)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x01)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x02)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x03)
	print(a)
except:
	print("No device")

print("------------------------------------")
try:
	print("Checking addr 3")
	a = BUS.write_byte(MUX_ADDR, 0b00000100)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x00)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x01)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x02)
	print(a)
	a = BUS.read_byte_data(DEV_ADDR, 0x03)
	print(a)
except:
	print("No device")
	
print("------------------------------------")
GPIO.cleanup()
