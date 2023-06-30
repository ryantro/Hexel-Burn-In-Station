"""
Module for the Hexel Burn In Station

This handles:
- Controlling the 3 micro-controllers on the laser driver board
- Setting I2C MUX address
- Reading info from I2C Device, AT240C
- Checking the state of the buttons
"""



import RPi.GPIO as GPIO
import time
import smbus
import glob
import serial
import socket
import threading
import select
import subprocess

#----------------------------------------------------------------------#

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
TEST_2 = False

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

#----------------------------------------------------------------------#

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
	
	def __init__(self) -> None:
		"""Initialize object"""
		
		self.addr = None
		self.ld_ser = None
		self.connected = False
		self.chan = None
		self.on = False
		self.sw_intr_lck = False
		self.volt = '0'	# Hexel Voltage
		self.curr = '0'	# Hexel Current Output Value
		self.curr_set = '0' # Hexel Current Set Value
		
		return None
	
	def __send(self, cmd: str) -> str:
		"""Method to send and recieve from Tom's board"""
		
		resp = ""
		try:
			# Test printing
			if(TEST_2):
				print("CHAN {} - Sending: ".format(self.chan) + cmd.replace('\r\n',''))
			
			# Encode and send command
			tosend = cmd.encode()
			self.ser.write(tosend)
			
			# Sleep to allow buffer to fill
			time.sleep(0.5)
			bts = self.ser.inWaiting()
			
			# Recieve return string
			resp = self.ser.read(bts) #_until(end.encode())
			resp = resp.decode()
			
			# Test printing
			if(TEST_2):
				print("CHAN {} - Recieved:\n  ".format(self.chan) + resp.replace("\n",'').replace("\r",''))
				pass
		
		except KeyboardInterrupt:
			raise KeyboardInterrupt("Keyboard Interrupt triggered in __send() method")
		
		except:
			print("CHAN {} - Failed to send command: ".format(self.chan) + cmd)
		
		return resp
	
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
				# INIT
				cmd = "INIT\r\n"
				self.__send(cmd)
				
				# WHOAMI
				cmd = "WHOAMI?\r\n"
				resps = self.__send(cmd) 
				self.chan = resps.split(' ')[-1].replace('\n','').replace('\r','')
				
			except:
				print("Failed to get channel.")
				self.connected = False
				self.close()
				
			return self.chan
			
		else:
			print("ERROR: No device connected.")
			return 0
	
	def getCurrent(self) -> None:
		"""Get the current across the laser diode"""
		
		if(self.connected == True):
			
			# Send command
			cmd = "READI?\r\n"
			resp = self.__send(cmd)
			
			# Parse output string
			resp = resp.split('=')[-1]
			curr = resp.replace(' ','').replace('\n','').replace('\r','')
			
			# Test printing
			if(TEST_2):
				print("  CHAN-" + self.chan + " Current: [" + curr + "]")
			
			# Set current
			if(self.__is_num(curr)):
				self.curr = curr
			else:
				self.curr = 'None'
				print("  CHAN-" + self.chan + " ERROR: Could not read current.")
				raise TomBoardError("Failed to read current.")
			
		else:
			print("ERROR: No device connected.")
		
		return

	def getVoltage(self) -> None:
		"""Get the voltage drop accross a hexel"""
		
		if(self.connected == True):
						
			# Send command
			cmd = "READLDV?\r\n"
			resp = self.__send(cmd)
			
			# Parse output string
			resp = resp.split('?')[-1]
			volt = resp.replace(' ','').replace('\n','').replace('\r','')
			
			# Check formatting
			if(self.__is_num(volt)):
				self.volt = volt
			else:
				self.volt = 'None'
				print("  CHAN-" + self.chan + " ERROR: Could not read voltage.")
				raise TomBoardError("Failed to read current.")
			
			# Test printing
			if(TEST_2):
				print("  CHAN-" + self.chan + " Voltage: [" + volt + "]")
			
		else:
			# No device connected
			print("ERROR: No device connected.")
			self.volt = None
			
		return
	
	def __is_num(self, num: str) -> bool:
		"""Check if the input string is a number"""
		
		# Replace negative sign and decimal point
		fnum = num.replace('-','').replace('.','')
		
		return fnum.isnumeric()
	
	def close(self):
		"""Close laser driver com port"""
		
		try:
			self.ser.close()
			
		except:
			print("Failed to close laser driver for chan {}.".format(self.chan))
			
	def laserOn(self):
		"""If the laser is off, turn it on"""
		
		# Cehck if the laser is off and if the sw interlock is on
		if(self.on == False and self.sw_intr_lck == True):
			
			# Print for testing
			if(TEST):
				print("Turning chan {} ON".format(self.chan))
			
			# Set state to on (True)
			self.on = True
			
			# Send command to turn laser on
			cmd = "SETI {}\r\n".format(self.curr_set)
			cmd = self.__send(cmd)
			
		return
		
	def laserOff(self):
		"""If the laser is on, turn it off"""
		
		# Check if the laser is on
		if(self.on == True):
			
			# Print for testing
			if(TEST):
				print("Turning chan {} OFF".format(self.chan))
				
			# Set state to off
			self.on = False
			
			# Send command to turn laser off
			cmd = "SETI 0\r\n"
			self.__send(cmd)
			
		return

#----------------------------------------------------------------------#

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
		
		# Initialize child object
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
		
	def __str__(self) -> str:
		"""String version of object
		Returns: 
			CHAN, BUTTON_STATE, HEXEL_SER, VOLTAGE, CURRENT
		"""
		return "{}, {}, {}, {}, {}".format(self.chan, self.button_state, 
											self.hexel_ser, self.volt, 
											self.curr)
	
	def get_button_state(self):
		"""Find what state the button is in"""
		self.button_state = GPIO.input(self.button_pin)
		if(True):
			print("Button state for chan {}: {}".format(self.chan, 
											self.button_state))
			
		return self.button_state
	
	def update(self):
		"""Read the state of the button and I2C"""
		
		############################
		# TODO: Check for i2c dev ##
		############################
		
		self.set_i2c_mux_and_read()
		
		if(self.get_button_state() == True): # and self.hexel_ser != None):
			if(self.sw_intr_lck == True):
				# If button to turn on laser is pressed
				self.laserOn()
				
			else:
				print("ERROR: Software interlock is not set.")
		
		else:
			# If button to turn laser off is unpressed
			self.laserOff()
		
		# Update voltage
		self.getVoltage()
		
		# Update current
		self.getCurrent()
		
		if(TEST):
			print(self.__str__())		
			
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
			if(TEST_2):
				# print("WARNING: No I2C device found for chan {}.".format(self.chan))
				pass
			self.hexel_ser = None
			return 0
		
		# Create empty array
		int_list = [0, 0, 0, 0]
		val = 0
		
		# Read serial number from I2C device
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
			# print("WARNING: Failed to read I2C device for chan {}.".format(self.chan))
			self.hexel_ser = None
			return 0
		
		return
	
#----------------------------------------------------------------------#
		
class Modules:
	"""Class that handles operations for all modules
	
	Attributes:
		modules (list): Holds all modules objects
		wd (float): Time that watchdog was triggered last
		max_time (float): Max time before watchdog shuts down lasers
	
	"""
	
	def __init__(self):
		"""initialize object"""
		self.modules = [] 			# Create empty list for modules
		self.wd = time.time() 		# Watchdog Timer Start Time
		self.max_time = 60*60*15	# Max time before triggering turn-off
		self.intr_lck = False		# Interlock
		self.lds_connected = False	# Variable to indicated if lds are connected
		self.eth_connected = False  # Variable to indicate of ethernet is connected 
		
		# Connect Tom's laser driver board
		# self.connect_ports()	
		# ^ Changed this to be called by above method
		
		# TCP/IP communication setup
		self.s = None
		
		return
	
	# Finds all ports with the ttyACM label	
	def connect_ports(self) -> None:
		"""Connect to all of Tom's laser driver boards"""
		
		# Create empty list for modules
		self.modules = []
		
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
						break
					
					# Attach object to current current object in for of a list
					self.modules.append(module)
					
			except (OSError, serial.SerialException):
				print("ERROR: Could not connect to port {}".format(port))
				pass	
		
		# Check length of module list
		if(len(self.modules) != 3):
			raise TomBoardError("TomBoardError: Only connected to {} out of boards.".format(len(self.modules)))
		
		# Sort the object list by laser driver channel
		self.modules.sort(key = lambda x: x.chan)
		
		# Indicate that laser drivers are all conncted
		self.lds_connected = True
		
		return None
	
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
	
	def set_all_currents(self, curr_set: str):
		"""Set the current set point for all lasers"""
		for module in self.modules:
			module.curr_set = curr_set
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
		if(len(self.modules) != 0):
			ret_str = []
			for module in self.modules:
				ret_str.append(str(module))
			return "\n".join(ret_str)
		else:
			# This condition is hit if communication is never established
			# with Tom's laser driver board.
			line = "None, None, None, None, None"
			ret_str = [line, line, line]
			return "\n".join(ret_str)

	def recv_and_send_loop(self):
		"""Recieve data from the main PC and send back status"""
		
		# Setup socket
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		
		while(self.eth_connected == False):
			try:
				# Bind socket
				self.s.bind((TCP_IP, TCP_PORT))
				self.s.listen(1)
				self.eth_connected = True
				
			except OSError as e:
				print(e)
				print("Failed to establish ethernet connection, trying again...")
				self.eth_connected = False
				time.sleep(1)
					
			except Exception as e:
				# CAtch all
				print(e)
				print("fdsafdsafdsafdsa")
				try:
					# Close the socket
					self.s.close()
				except:
					pass
					
				return
		
		while(True):
			try:
				# Create a waitable object with a timeout of 1 second
				readable, _, _ = select.select([self.s], [], [], 1.0)
				
				if(TEST_2):
					print("Waiting for command")
				
				if(self.s in readable):
					
					# Wait for connection
					self.conn, addr = self.s.accept()
					
					if(TEST_2):
						print ('Connection address:', addr)
						
					# Recieve value from PC
					recv_val = self.conn.recv(BUFFER_SIZE).decode()				
					
					if("ON" in recv_val):
						curr_set = recv_val.split(" ")[-1]
						if(TEST_2):
							print("Current set point set to {} A".format(curr_set))
						
						# Set the software interlock on
						self.set_sw_intr_lck_on_all()
						
						# Set the current setpoints
						self.set_all_currents(curr_set)
						
					else:
						if(TEST):
							print("SW interlock off")
							
						# Set the software interlock off
						self.set_sw_intr_lck_off_all()
					
					if(TEST_2):
						print("received data:", recv_val)
					
					# Send out status update with latest data
					data = self.__str__().encode()	# Generate string
					self.conn.send(data)  			# Send via TCP/IP
					
					# Set new watchdog start time
					self.wd = time.time()				
			
			except Exception as e:
				# Close TCP/IP connection
				print("432132143214321432143214321/nfdsafdsafdsafdsafdsafa")
				print(e)
				
				# Exit loop				 
				break
		
		try:
			# Close the socket
			self.s.close()
			
		except Exception:
			pass
		
		return

	def reset_usb(self) -> None:
		"""Turn on and off all USB connections and re-connect to laser drivers"""
		
		# Mark connected as false
		self.lds_connected = False
		
		# Try to close all connections
		self.close_all()
		
		# Call bash script to restart all usb devices 
		print(subprocess.run(["./usb-reset.bash"], shell=True))
		
		# Give some time for devices to re-initialize
		print("Sleeping 15 seconds...")
		time.sleep(15)
		
		# Re-Connect to all the ports
		self.connect_ports()
		
		return None

	def close_all(self):
		"""Close all serial ports"""
		for module in self.modules:
			try:
				module.close()
			except:
				print("ERROR: Failed to close module {}".format(module.chan))

		return
				
#----------------------------------------------------------------------#		

class TomBoardError(Exception):
	"""Exception to handle errors relating to poor firmware on Tom's 
	laser driver board"""
	pass

#----------------------------------------------------------------------#	

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
		ms.connect_ports()
		
	except TomBoardError:
		ms.lds_connected = False
		while(ms.lds_connected == False):
			try:
				ms.reset_usb()
				
			except TomBoardError as e:
				print(e)
				print("---------------------------------------------")
				pass
				
	try:
		if(ms.lds_connected):
			# Set all currents to 0
			ms.set_all_currents('0')
			
			if(TEST):
				print("Enabling all interlocks.")
				ms.set_sw_intr_lck_on_all()
			
			# Loop to handle updating states of laser drivers
			while(True):
				
				try:
					ms.update()
					time.sleep(0.1)
					
				except TomBoardError as e:
					ms.lds_connected = False
					while(ms.lds_connected == False):
						# Try to reset usb and reconnect to Tom's board
						try:
							ms.reset_usb()
							if(TEST):
								print("Enabling all interlocks.")
								ms.set_sw_intr_lck_on_all()
							
						except TomBoardError as e:
							print(e)
							print("---------------------------------------------")
							pass
					
					# ms.reset_usb()
	
	except KeyboardInterrupt:
		print("\n\nKeyboard interrupt triggered!\nClosing program.\n")
		# ms.s.close()
		print("Socket closed.")
		
	except Exception as e: 
		print("Catch all excepting hit:")
		print(str(e))
		
	finally:
		# Shut down the socket
		ms.set_all_currents('0')
		
		if(ms.s != None):
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

#----------------------------------------------------------------------#

main()

#----------------------------------------------------------------------#
