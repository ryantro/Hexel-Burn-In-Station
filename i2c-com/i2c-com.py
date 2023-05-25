import smbus
import time



class AT240C:
	def __init__(self):
		# AT240C device address
		self.devAddr = 0x50
		
		# Data adresses being used
		self.dataAddrs = [0x00, 0x01, 0x02, 0x03]

		# Initialize I2C (SMBus)
		self.bus = smbus.SMBus(1)
		
		return
	
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




		
A = AT240C()
A.write(1712)
A.read()
