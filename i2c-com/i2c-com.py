








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

MUX_ADDR = 0x70
M_1 = 0x00
DEV_ADDR = 0x50

class AT240C:
	def __init__(self):
		# AT240C device address
		self.devAddr = 0x50
		
		# Data adresses being used
		self.dataAddrs = [0x00, 0x01, 0x02, 0x03]

		# Initialize I2C (SMBus)
		self.bus = smbus.SMBus(1)
		
		self.i2c_addr = [0, 0, 0]
		
		return
	
	
	def check(self, l = [0, 0, 0]):
		"""Sets the I2C MUX address and
		tries to read the value on the AT240C
		"""
		self.i2c_addr = l
		
		# Set GPIO MUX Pins
		GPIO.output(MUX_0, self.i2c_addr[0])
		GPIO.output(MUX_1, self.i2c_addr[1])
		GPIO.output(MUX_2, self.i2c_addr[2])
		time.sleep(0.01)
		
		# Check if I2C device is connected
		try:
			print("Write #1")
			a = self.bus.read_byte_data(MUX_ADDR)
			print("Write #2")
			a = self.bus.read_byte_data(DEV_ADDR, 0x00)
			print(a)
			print("SUCCESS!")
			return 1
			
		except:
			print("ERROR: No I2C device found for chan.")
			#print(self.i2c_addr)
			self.hexel_ser = 0
			return 0
	
	def write(self, int_val, dev = 0):
		"""
		Write data represented by 4 bytes.
		"""
		# TODO:
		# SELECT I2C MUX
		
		# Convert int to byte
		bytes_val = int_val.to_bytes(4, 'big')

		# Convert bytes to list of bytes
		byte_list = [bytes_val[i] for i in range(0, len(bytes_val), 1)]

		# Write value to at240c
		try:
			for i in range(0,len(self.dataAddrs)):
				time.sleep(0.01)
				self.bus.write_byte_data(self.devAddr, self.dataAddrs[i], byte_list[i]) 
			print("Write successful :)")
			return 0
		except:
			print("Write failed :(")
			return -1
		
		return
		
	def read(self):
		"""
		Read data from AT240C and convert it to int.
		"""
		# Create empty array

		int_list = [0, 0, 0, 0]
		val = 0
		# Read i2c
		try:
			for i in range(0, len(self.dataAddrs)):
				time.sleep(0.01)
				
				# Read from AT240C
				int_list[i] = self.bus.read_byte_data(self.devAddr, self.dataAddrs[i]) 
							
				# Convert int list to value
				shift = 8 * (3 - i)
				val = (int_list[i] << shift) + val
			print(val)
			return val
		except:
			print("ERROR: Failed to read")
			return 0



try: 
	A = AT240C()
	A.check()
finally:
	# Cleanup GPIO
	GPIO.cleanup()
#A.write(1712)
#A.read()
