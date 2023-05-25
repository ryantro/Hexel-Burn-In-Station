"""
Module for the Hexel Burn In Station

This handles:
- Controlling the 3 micro-controllers on the laser driver board
- Setting I2C MUX address
- Reading info from I2C Device, AT240C
- Checking the state of the buttons
"""

__version__ = '0.1'
__author__ = 'Ryan Robinson'

import RPi.GPIO as GPIO
import time
import smbus
import glob
import serial

# Configure GPIO
GPIO.setmode(GPIO.BCM)

# On/Off Buttons
BUTTON_0 = 5
BUTTON_1 = 6
BUTTON_2 = 13

# Configure GPIO Pins To Input Pullup
GPIO.setup(BUTTON_0, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Pins For I2C Mux Address 
MUX_0 = 23
MUX_1 = 24
MUX_2 = 25

# Configure GPIO Pins To Output
GPIO.setup(MUX_0, GPIO.OUT)
GPIO.setup(MUX_1, GPIO.OUT)
GPIO.setup(MUX_2, GPIO.OUT)

# Intialize I2C Bus
I2C_CHAN = 1
BUS = smbus.SMBus(I2C_CHAN)

# Number of devices connected
DEVICES = 3 

# I2C Mux Addresses
I2C_0_ADDR = [0, 0, 0] 
I2C_1_ADDR = [0, 0, 1]
I2C_2_ADDR = [0, 1, 0]

# AT240C device address
DEV_ADDR = 0x50

# Data adresses being used
DATA_ADDRS = [0x00, 0x01, 0x02, 0x03]

# Printing
TEST = True


class Laser:
	def __init__(self):
		self.addr = None
		self.ld_ser = None
		self.connected = False
		self.chan = None
	
	def open(self, addr):
		"""
		Open com port of laser driver
		"""
		try:
			# Open device on addr com port
			self.ser = serial.Serial(addr, 115200, timeout = 3)
			
			# Update info
			self.addr = addr
			self.connected = True
			
			# Get the channel
			self.findChan()
			print("Connected to laser on channel {}.".format(self.chan))
		except:
			self.connected = False
			
		return
	
	def findChan(self):
		"""
		Get channel of laser driver
		"""
		if(self.connected == True):
			try:
				cmd = "WHOAMI\r\n".encode()
				self.ser.write(cmd)
				resps = self.ser.read_until(">>>".encode()).decode()
				self.chan = resps.split('\n')[1].split(' ')[3].split('\\')[0]
			except:
				print("Failed to get channel")
				self.connected = False
				self.close()
			return self.chan
		else:
			print("ERROR: No device connected.")
			return 0
	
	def getVoltage(self):
		if(self.connected == True):
			volt = 0
			try:
				cmd = "READLDA?\r\n".encode()
				self.ser.write(cmd)
				resps = self.ser.read_until(">>>".encode()).decode()
				print(resps)
				#volt = resps.split('\n')[1].split(' ')[3].split('\\')[0]
				#print(volt)
			except:
				print("Failed to read voltage drop")
			return volt
		else:
			print("ERROR: No device connected.")
			return 0
	
	def close(self):
		"""
		Close laser driver com port
		"""
		try:
			self.ser.close()
		except:
			print("Failed to close laser driver")
			
	def laserOn(self):
		print("placeholder for turning the laser on...")
		return
		
	def laserOff(self):
		print("placeholder for turning the laser off...")
		return

class Module(Laser):
	def __init__(self):
		
		Laser.__init__(self)
		
		# To report
		# self.chan = 0			# Laser Driver Channel
		# self.ld_ser = 0		# Laser Driver Serial Number
		self.hexel_ser = None	# Hexel Serial Number
		self.volt = 0			# Hexel Voltage
		self.curr = 0			# Hexel Current
		
		# Internal
		self.port = None		# Laser Driver Port
		self.button_pin = None	# Rasberry PI GPIO Pin
		self.button_state = True 		# Button Not pressed
		self.i2c_addr = [0, 0, 0]		# I2C Mux Addr
		
		return
		
	def __str__(self):
		"""String version of object"""
		return "{}, {}, {}, {}, {}".format(self.chan, self.ld_ser, 
											self.hexel_ser, self.volt, 
											self.curr)
	
	def get_button_state(self):
		"""Find what state the button is in"""
		self.button_state = GPIO.input(self.button_pin)
		return
	
	def set_i2c_mux_and_read(self):
		"""Sets the I2C MUX address and
		tries to read the value on the AT240C
		"""
		# Set GPIO MUX Pins
		GPIO.output(MUX_0, self.i2c_addr[0])
		GPIO.output(MUX_1, self.i2c_addr[1])
		GPIO.output(MUX_2, self.i2c_addr[2])
		time.sleep(0.01)
		
		# Check if I2C device is connected
		try:
			BUS.read_byte(DEV_ADDR)
			
		except:
			print("ERROR: No I2C Device Found")
			return 0
		
		# Create empty array
		int_list = [0, 0, 0, 0]
		val = 0
		try:
			for i in range(0, len(DATA_ADDRS)):
				time.sleep(0.01)
				
				# Read from AT240C
				int_list[i] = BUS.read_byte_data(DEV_ADDR, DATA_ADDRS[i]) 
							
				# Convert int list to value
				shift = 8 * (3 - i)
				val = (int_list[i] << shift) + val
			
			if(TEST):
				print(val)
			
			# Assign val to hexel serial
			self.hexel_ser = val
			return val
			
		except:
			print("ERROR: Failed to read I2C Device")
			return 0
		
		return
	

		
class Modules:
	def __init__(self):
		
		self.modules = [] 	# Create empty list for modules
		
		self.connect_ports()	# Connect Tom's laser driver board
		
		return
	
	# Finds all ports with the ttyACM label	
	def connect_ports(self):
		"""Connect to all of Tom's laser driver boards"""
		ports = glob.glob('/dev/tty[A-Za-z]*')

		for port in ports:
			try:
				if('ttyACM' in port):
					# Instatiate Module Obkect
					module = Module()
					
					# Open serial port to laser driver
					module.open(port)
					
					# Find what channel is connected to what
					if(module.chan == "1"):
						module.button_pin = BUTTON_0	# Rasberry PI GPIO Pin
						module.i2c_addr = I2C_0_ADDR 	# I2C Mux Addr
					elif(module.chan == "2"):
						module.button_pin = BUTTON_1	# Rasberry PI GPIO Pin
						module.i2c_addr = I2C_1_ADDR 	# I2C Mux Addr
					elif(module.chan == "3"):
						module.button_pin = BUTTON_2	# Rasberry PI GPIO Pin
						module.i2c_addr = I2C_2_ADDR 	# I2C Mux Addr
					else:
						print("ERROR: Module channel listed as {}.".format(module.chan))
					
					# Attach object to current current object in for of a list
					self.modules.append(module)
					
			except (OSError, serial.SerialException):
				print("ERROR: Could not connect to port {}".format(port))
				pass	
				
		return
	
	def update(self):
		"""Check button state"""
		
		# TODO: CHECK WATCHDOG TIMER
		
			# TODO: CHECK BUTTON STATE
		
				# TODO: CHECK I2C ADDR
		
					# TODO: TURN DEVICE ON
			
		
		
		return
	
	def read_all_buttons(self):
		for module in self.modules:
			module.get_button_state()
		return
	
	def read_all_hexel_sers(self):
		"""Read all hexel serial numbers"""
		for module in self.modules:
			module.set_i2c_mux_and_read()
		return
	
	def __str__(self):
		"""Return data in string format"""
		ret_str = []
		for module in self.modules:
			ret_str.append(str(module))
		
		return "\n".join(ret_str)

	def close_all(self):
		"""Close all serial ports"""
		for module in self.modules:
			try:
				module.close()
			except:
				print("ERROR: Failed to close module {}".format(module.chan))
				




def main():
	
	print("run")	

	try:
		ms = Modules()
		ms.read_all_hexel_sers()
		print(ms)

	finally:
		ms.close_all()
		GPIO.cleanup()
	

	

	
	
	# Connect To Laser Drivers


main()

