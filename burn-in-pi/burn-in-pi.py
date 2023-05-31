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
import socket
import threading
import select

# Configure GPIO
GPIO.setmode(GPIO.BCM)

# On/Off Buttons
BUTTON_0 = 26
BUTTON_1 = 6
BUTTON_2 = 5

# Configure GPIO Pins To Input Pullup
GPIO.setup(BUTTON_0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BUTTON_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BUTTON_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Pins For I2C Mux Address 
MUX_0 = 25
MUX_1 = 24
MUX_2 = 23

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
CHAN_ARRAY = [0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000]

# AT240C device address
MUX_ADDR = 0x70
DEV_ADDR = 0x50

# Data adresses being used
DATA_ADDRS = [0x00, 0x01, 0x02, 0x03]

# Variable to enable or disable all print statements
TEST = True

# Find static IP set in dhcpcd.conf file
with open('/etc/dhcpcd.conf') as fin :
    lines = fin.readlines()
line_num = -1
for k,line in enumerate(lines) :
	# print(line)
	if "static ip_address" in line:
		line_num = k
TCP_IP = str(lines[line_num].split("=")[-1].replace(" ", "").replace("\n", ""))

# TCP IP configuration settings
#TCP_IP = '192.168.0.169' # Pi IP
TCP_PORT = 9999
BUFFER_SIZE = 1024

# SETPOINTS
CURRENT_SET = 3.5

class Laser:
	"""Class to control everything with respect to a single channel on
	the laser driver board
	
	Attrbutes:
		addr (str): Address of the laser driver board
		ld_ser (int): Laser driver board serial number
		connected (bool): Connection status
		chan (int): Channel of the laser driver board
		on (bool): Laser on/off status
		
	"""
	
	def __init__(self):
		"""Initialize object"""
		self.addr = None
		self.ld_ser = None
		self.connected = False
		self.chan = None
		self.on = False
		self.sw_intr_lck = False
		self.volt = 0	# Hexel Voltage
		self.curr = 0	# Hexel Current
	
	def open(self, addr):
		"""Open com port of laser driver"""
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
		"""Get channel of laser driver"""
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
		"""Get the voltage drop accross a hexel"""
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
		"""Close laser driver com port"""
		try:
			self.ser.close()
		except:
			print("Failed to close laser driver for chan {}.".format(self.chan))
			
	def laserOn(self):
		"""If the laser is off, turn it on"""
		
		if(self.on == False and self.sw_intr_lck == True):
			print("placeholder for turning chan {} laser on...".format(self.chan))
			self.on = True
			self.curr = CURRENT_SET
			# self.ser.write(cmd_to_set_current_to_3.5)
			# self.volt = self.getVoltage()
		return
		
	def laserOff(self):
		"""If the laser is on, turn it off"""
		
		if(self.on == True):
			print("placeholder for turning chan {} laser off...".format(self.chan))
			self.on = False
			self.curr = 0
			# self.ser.write(cmd_to_set_current_to_0)
			# self.volt = 0
		return

class Module(Laser):
	"""Class that handles a single module.
	
	Attributes:
		hexel_ser (int): Hexel serial number.
		volt (float): Voltage drop accross hexel.
		curr (float): Current setpoint.
		button_pin (int): Pin corresponding to on/off button.
		button_state (bool): Last recorded state of the button.
		i2c_addr (list): MUX address for I2C device.
	
	"""
	
	def __init__(self):
		"""Initialize object"""
		Laser.__init__(self)
		
		# To report
		# self.chan = 0			# Laser Driver Channel
		# self.ld_ser = 0		# Laser Driver Serial Number
		self.hexel_ser = None	# Hexel Serial Number
		
		# Internal
		# self.port = None		# Laser Driver Port
		self.button_pin = None	# Rasberry PI GPIO Pin
		self.button_state = True 		# Button Not pressed
		self.i2c_addr = 0b00000001		# I2C Mux Addr
		
		return
		
	def __str__(self):
		"""String version of object"""
		return "{}, {}, {}, {}, {}".format(self.chan, self.ld_ser, 
											self.hexel_ser, self.volt, 
											self.curr)
	
	def get_button_state(self):
		"""Find what state the button is in"""
		self.button_state = GPIO.input(self.button_pin)
		if(False):
			print("Button state for chan {}: {}".format(self.chan, 
											self.button_state))
			
		return self.button_state
	
	def update(self):
		"""Read the state of the button and I2C"""
		if(self.get_button_state() == True):
			if(self.sw_intr_lck == True):
				# If button to turn on laser is pressed
				self.set_i2c_mux_and_read()
				self.laserOn()
			else:
				print("ERROR: Software interlock is not set.")
		else:
			# If button to turn laser off is unpressed
			self.laserOff()		
			
		return
	
	def set_sw_intr_lck_on(self):
		"""Turn the software interlock on (enable laser firing)"""
		# Set software interlock variable to true
		self.sw_intr_lck = True
		
		return
	
	def set_sw_intr_lck_off(self):
		"""Turn the software interlock off (disable laser firing)"""
		# Set software interlock variable to false
		self.sw_intr_lck = False
		
		# Ensure that the laser is turned off
		self.laserOff()
		
		return
	
	def set_i2c_mux_and_read(self):
		"""Sets the I2C MUX address and
		tries to read the value on the AT240C
		"""
		# Set GPIO MUX
		BUS.write_byte(MUX_ADDR, self.i2c_addr)
		
		time.sleep(0.01)
		
		# Check if I2C device is connected
		try:
			BUS.read_byte(DEV_ADDR)
			
		except:
			print("ERROR: No I2C device found for chan {}.".format(self.chan))
			print(self.i2c_addr)
			self.hexel_ser = 0
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
			
			if(False):
				print(val)
			
			# Assign val to hexel serial
			self.hexel_ser = val
			return val
			
		except:
			print("ERROR: Failed to read I2C device for chan {}.".format(self.chan))
			self.hexel_ser = None
			return 0
		
		return
	

		
class Modules:
	"""Class that handles operations for all modules
	
	Attributes:
		modules (list): Holds all modules objects
		wd (float): Time that watchdog was triggered last
		max_time (float): Max time before watchdog shuts down lasers
	
	"""
	
	def __init__(self):
		"""initialize object"""
		self.modules = [] 		# Create empty list for modules
		self.wd = time.time() 	# Watchdog Timer Start Time
		self.max_time = 10 # 60*5	# Max time before triggering turn-off
		self.intr_lck = False	# Interlock
		
		# Connect Tom's laser driver board
		self.connect_ports()	
		
		# TCP/IP communication setup
		self.s = None
		
		return
	
	# Finds all ports with the ttyACM label	
	def connect_ports(self):
		"""Connect to all of Tom's laser driver boards"""
		
		# Initialize seen_devices at 0
		seen_devices = 0
		
		# Create empty port list
		ports = []
		
		# Wait for all devices to be seen
		while(seen_devices != DEVICES):
			# Loop delay
			time.sleep(0.1)
			
			# Reset seen_devices
			seen_devices = 0
			
			if(TEST):
				print("Waiting to recognize all devices...")
			
			# List all ports
			ports = glob.glob('/dev/tty[A-Za-z]*')
			
			# List number of seen_devices
			for port in ports:
				if('ttyACM' in port):
					seen_devices = seen_devices + 1
		
		if(TEST):
			print("Found 3 devices... Proceeding to connect.")
		
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
						module.i2c_addr = CHAN_ARRAY[0]	# I2C Mux Addr
					elif(module.chan == "2"):
						module.button_pin = BUTTON_1	# Rasberry PI GPIO Pin
						module.i2c_addr = CHAN_ARRAY[1]	# I2C Mux Addr
					elif(module.chan == "3"):
						module.button_pin = BUTTON_2	# Rasberry PI GPIO Pin
						module.i2c_addr = CHAN_ARRAY[2]	# I2C Mux Addr
					else:
						print("ERROR: Module channel listed as {}.".format(module.chan))
					
					# Attach object to current current object in for of a list
					self.modules.append(module)
					
			except (OSError, serial.SerialException):
				print("ERROR: Could not connect to port {}".format(port))
				pass	
		
		# Sort the object list by laser driver channel
		self.modules.sort(key = lambda x: x.chan)
				
		return
	
	def update(self):
		"""Check button state"""
		# Check the watchdog timer
		if(time.time() - self.wd < self.max_time):
		
			# If the watchdog timer is satisfied, update modules
			for module in self.modules:
				module.update()
				
		else:
			# Turn off all devices if watchdog has not been reset
			if(TEST):
				print("WD Time Passed")
			
			self.turn_all_off()
		
		return
	
	def turn_all_off(self):
		"""Turn all lasers off"""
		for module in self.modules:
			module.laserOff()		
		return
	
	def read_all_buttons(self):
		"""Read all module buttons"""
		for module in self.modules:
			module.get_button_state()
		return
	
	def read_all_hexel_sers(self):
		"""Read all hexel serial numbers"""
		for module in self.modules:
			module.set_i2c_mux_and_read()
		return
	
	def set_sw_intr_lck_on_all(self):
		"""Set the software interlock on for all lasrs"""
		for module in self.modules:
			module.set_sw_intr_lck_on()
		return
	
	def set_sw_intr_lck_off_all(self):
		"""Set the software interlock off for all lasrs"""
		for module in self.modules:
			module.set_sw_intr_lck_off()
		return
	
	def __str__(self):
		"""Return data in string format"""
		ret_str = []
		for module in self.modules:
			ret_str.append(str(module))
		
		return "\n".join(ret_str)

	def recv_and_send_loop(self):
		"""Recieve data from the main PC and send back status"""
		# Create and setup socket
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.s.bind((TCP_IP, TCP_PORT))
		self.s.listen(1)
		
		while(True):
			try:
				# Create a waitable object with a timeout of 1 second
				readable, _, _ = select.select([self.s], [], [], 1.0)
				
				if(TEST):
					print("Waiting for command")
				
				if(self.s in readable):
					
					# Wait for connection
					self.conn, addr = self.s.accept()
					
					if(TEST):
						print ('Connection address:', addr)
						
					# Recieve value from PC
					recv_val = self.conn.recv(BUFFER_SIZE).decode()
					
					if(recv_val == "ON"):
						if(TEST):
							print("SW interlock on")
						
						# Set the software interlock on
						self.set_sw_intr_lck_on_all()
						
					else:
						if(TEST):
							print("SW interlock off")
							
						# Set the software interlock off
						self.set_sw_intr_lck_off_all()
					
					if(TEST):
						print("received data:", recv_val)
					
					# Send out status update with latest data
					data = self.__str__().encode()	# Generate string
					self.conn.send(data)  			# Send via TCP/IP
					
					# Set new watchdog start time
					self.wd = time.time()
					
			except Exception as e:
				# Close TCP/IP connection
				print(e)
				
				# Exit loop				 
				break
		
		try:
			# Close the socket
			self.s.close()
			
		except Exception:
			pass
		
		return

	def close_all(self):
		"""Close all serial ports"""
		for module in self.modules:
			try:
				module.close()
			except:
				print("ERROR: Failed to close module {}".format(module.chan))

		return
				

		

def main():
	"""Method to run the main program"""
	# Instantiate modules object
	ms = Modules()

	# Create thread for TCP/IP communication
	tcp_ip_thread = threading.Thread(target = ms.recv_and_send_loop)
	
	# Start thread
	tcp_ip_thread.start()
	
	try:
		# Update loop
		while(True):
			ms.update()
			time.sleep(0.1)
			
	except KeyboardInterrupt:
		print("\n\nKeyboard interrupt triggered!\nClosing program.\n")
		ms.s.close()
		print("Socket closed.")
		
	except Exception as e: 
		print("Catch all excepting hit:")
		print(str(e))
		
	finally:
		# Shut down the socket
		ms.s.close()
		
		if(tcp_ip_thread.is_alive()):
			if(TEST):
				print("TCP/IP thread alive...")
				print("Joining thread...")
				
			tcp_ip_thread.join()
			
			if(TEST):
				print("Thread succesfully joined.")
	
	try:
		# Close all laser driver coms
		ms.close_all()
	except:
		pass
	
	try:	
		# Cleanup GPIO
		GPIO.cleanup()
	except:
		pass

main()

