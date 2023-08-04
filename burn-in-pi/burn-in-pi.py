#!/usr/bin/env python3

__author__ = "Ryan Robinson"
__version__ = "1.0.1"
__maintainer__ = "Ryan Robinson"
__email__ = "ryan.robinson@nuburu.net"
__status__ = "Production"

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

# ----------------------------------------------------------------------#

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
CHAN_ARRAY = [0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000]

# AT240C device address
MUX_ADDR = 0x70
DEV_ADDR = 0x50

# Data adresses being used
DATA_ADDRS = [0x00, 0x01, 0x02, 0x03]

# Variable to enable or disable all print statements
TEST = True
TEST_2 = False

# Find static IP set in dhcpcd.conf file
with open('/etc/dhcpcd.conf') as fin:
	lines = fin.readlines()
line_num = -1
for k, line in enumerate(lines):
	# print(line)
	if "static ip_address" in line:
		line_num = k
TCP_IP = str(lines[line_num].split("=")[-1].replace(" ", "").replace("\n", ""))

# TCP IP configuration settings
# TCP_IP = '192.168.0.169' # Pi IP
TCP_PORT = 9999
BUFFER_SIZE = 1024

# SETPOINTS
CURRENT_SET = 3.5

# MAXIMUM ATTEMPTS
ATTEMPTS = 2


# ----------------------------------------------------------------------#

def usb_reset() -> None:
	"""Reset all usb devices"""
	# Reset program
	prgm = '/home/lab/Documents/Python/burn-in-pi/usb-reset.bash'
	print(subprocess.run([prgm], shell=True))

	# Give some time for devices to re-initialize
	print("Sleeping 15 seconds...")
	time.sleep(15)

	return None


# ----------------------------------------------------------------------#

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
		self.volt = 'None'  # '0'	# Hexel Voltage
		self.curr = 'None'  # '0'	# Hexel Current Output Value
		self.curr_set = 'None'  # '0' # Hexel Current Set Value

		return

	def __send(self, cmd: str, end=None, lines=None, attempt=0) -> str:
		"""Method to send and recieve from Tom's board"""
		# time.sleep(0.05)
		resp = ""
		try:
			# Reset the input buffer
			self.ser.reset_input_buffer()

			# Test printing
			if (TEST_2):
				print("CHAN {} - Sending: ".format(self.chan) + cmd.replace('\r\n', ''))

			# Encode and send command
			tosend = cmd.encode()
			self.ser.write(tosend)

			# Sleep to allow buffer to fill
			if end == None and lines == None:
				# If no end marker given
				time.sleep(0.4)
				bts = self.ser.inWaiting()
				print(bts)
				resp = self.ser.read(bts)  # _until(end.encode())

			elif end != None and lines == None:
				# Parse till end marker
				resp = self.ser.read_until(end.encode())

			elif lines != None:
				# Parse number of lines
				resp = ''.encode()
				for i in range(0, lines):
					if end == None:
						resp = resp + self.ser.read_until('\r\n'.encode())

					else:
						resp = resp + self.ser.read_until(end.encode())

					if TEST_2:
						print(i)
						print(resp)

			# For finding end characters
			if TEST_2:
				print(resp)

			# Decode the response
			resp = resp.decode()

			# Test printing
			if TEST_2:
				print("CHAN {} - Recieved:\n  ".format(self.chan) + resp.replace("\n", '').replace("\r", ''))
				pass

		except KeyboardInterrupt:
			raise KeyboardInterrupt("Keyboard Interrupt triggered in __send() method")

		except:
			if attempt >= ATTEMPTS:
				print("CHAN {} - Failed to send command: ".format(self.chan) + cmd)

			else:
				resp = self.__send(cmd, end=end, lines=lines, attempt=attempt + 1)

		return resp

	def open(self, addr):
		"""Open com port of laser driver"""

		try:
			# Open device on addr com port
			self.ser = serial.Serial(addr, 115200, timeout=1)

			# Update info
			self.addr = addr
			self.connected = True

			# Reset the input buffer
			self.ser.reset_input_buffer()

			# Get the channel
			self.findChan()
			print("Connected to laser on channel {}.".format(self.chan))
		except:
			self.connected = False

		return

	def findChan(self):
		"""Get channel of laser driver"""

		if self.connected == True:
			try:
				# INIT
				cmd = "INIT\r\n"
				self.__send(cmd, end='\r\n\r\n')

				# WHOAMI
				cmd = "WHOAMI?\r\n"
				resps = self.__send(cmd, end='\r\r\n')
				self.chan = resps.split(' ')[-1].replace('\n', '').replace('\r', '')

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

		# TODO: Wait for Tom to implement current get command

		return None

		"""
		if self.connected == True:

			# Send command
			cmd = "READI?\r\n"
			resp = self.__send(cmd)

			# Parse output string
			resp = resp.split('=')[-1]
			curr = resp.replace(' ', '').replace('\n', '').replace('\r', '')

			# Test printing
			if (TEST_2):
				print("  CHAN-" + self.chan + " Current: [" + curr + "]")

			# Set current
			if (self.__is_num(curr)):
				self.curr = curr
			else:
				self.curr = 'None'
				print("  CHAN-" + self.chan + " ERROR: Could not read current.")
				raise TomBoardError("Failed to read current.")

		else:
			print("ERROR: No device connected.")
			
		return None
		"""

	def getVoltage(self, attempt=0) -> None:
		"""Get the voltage drop accross a hexel"""

		if self.connected == True:

			# Not implimented
			# self.volt = 'None'
			# return

			# Send command
			cmd = "READLDV?\r\n"
			resp = self.__send(cmd, lines=3)

			# Parse output string
			resp = resp.split('=')[-1]
			volt = resp.replace(' ', '').replace('\n', '').replace('\r', '')

			# Check formatting
			if self.__is_num(volt):
				v_float = float(volt)
				# Exaple value - 0.5697555
				self.volt = '{:.7f}'.format(v_float)
			# self.volt = volt

			# Section to try again if voltage reading fails
			else:
				if attempt >= ATTEMPTS:
					# If on the 2nd attempt or more
					self.volt = 'None'
					print("  CHAN-" + self.chan + " ERROR: Could not read voltage.")
					raise TomBoardError("Failed to read current.")

				else:
					self.getVoltage(attempt=attempt + 1)

			# Test printing
			if (TEST_2):
				print("  CHAN-" + self.chan + " Voltage: [" + volt + "]")

		else:
			# No device connected
			print("ERROR: No device connected.")
			self.volt = 'None'

		return

	def __is_num(self, num: str) -> bool:
		"""Check if the input string is a number"""

		# Replace negative sign and decimal point
		fnum = num.replace('-', '').replace('.', '')

		return fnum.isnumeric()

	def r_close(self):
		"""Close the laser driver w/o shutting off power"""
		try:
			self.ser.close()

		except:
			print("Failed to close laser driver for chan {}.".format(self.chan))

	def close(self):
		"""Close laser driver com port"""

		# Turn the laser off
		try:
			self.laserOff()
		except:
			pass

		try:
			self.ser.close()

		except:
			print("Failed to close laser driver for chan {}.".format(self.chan))

	def laserOn(self):
		"""If the laser is off, turn it on"""

		if (self.curr_set == 'None'):
			return

		# Cehck if the laser is off and if the sw interlock is on
		elif self.curr != self.curr_set and self.sw_intr_lck == True:

			# Till Tom's get current command is implimented
			self.curr = self.curr_set

			# Print for testing
			if TEST_2:
				print("Turning chan {} ON".format(self.chan))

			# Set state to on (True)
			self.on = True

			# Send command to turn laser on
			cmd = "SETI {}\r\n".format(self.curr_set)
			self.__send(cmd, end='OK\r\n')

		return

	def laserOff(self):
		"""If the laser is on, turn it off"""

		# Till Tom's get current is implimented
		self.curr = 0.0

		# Print for testing
		if (TEST_2):
			print("Turning chan {} OFF".format(self.chan))

		# Set state to off
		self.on = False

		# Send command to turn laser off
		cmd = "SETI 0\r\n"
		self.__send(cmd, end='OK\r\n')

		return


# ----------------------------------------------------------------------#

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
		# self.chan = 0				# Laser Driver Channel
		# self.ld_ser = 0			# Laser Driver Serial Number
		self.hexel_ser = 'None'  	# Hexel Serial Number
		self.i2c_addr = 0b00000001  # I2C Mux Addr

		# Internal
		# self.port = None			# Laser Driver Port
		self.button_pin = 'None'  	# Rasberry PI GPIO Pin
		self.button_state = 0  		# Button Not pressed
		self.__concurrent = 0 		# Consecutive serial reads
		self.__c_checks = 1

		return

	def __str__(self) -> str:
		"""String version of object
		Returns:
			CHAN, BUTTON_STATE, HEXEL_SER, VOLTAGE, CURRENT
		"""
		return "{}, {}, {}, {}, {}".format(self.chan, self.button_state,
										   self.hexel_ser, self.volt,
										   self.curr)

	def clear(self) -> None:
		"""Clear variables from Tom's board"""
		self.chan = 'None'
		# self.button_state = 'None'
		# self.hexel_ser = 'None'
		# self.volt = 'None'
		# self.curr = 'None'
		return None

	def get_button_state(self):
		"""Find what state the button is in"""
		self.button_state = GPIO.input(self.button_pin)
		return self.button_state

	def update(self):
		"""Read the state of the button and I2C"""

		############################
		# TODO: Check for i2c dev ##
		############################

		self.set_i2c_mux_and_read()

		if self.get_button_state() == True:  # and self.hexel_ser != None):
			if self.sw_intr_lck == True:
				# If button to turn on laser is pressed
				self.laserOn()

			else:
				print("ERROR: Software interlock is not set.")
				self.laserOff()

		else:
			# If button to turn laser off is unpressed
			self.laserOff()

		# Update voltage
		self.getVoltage()

		# Update current
		# TODO : Needs to be implimented
		# self.getCurrent()

		if self.chan != '3':
			print('[' + self.__str__().replace('\n', '') + '], ', end='')
		else:
			print('[' + self.__str__() + ']')

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

	def __set_ser(self, hexel_ser) -> None:
		"""Sets the serial number if it reads the same number multiple times"""

		if self.__concurrent >= self.__c_checks:
			self.hexel_ser = hexel_ser
			self.__concurrent = 0

		else:
			self.__concurrent = self.__concurrent + 1

		return None

	def set_i2c_mux_and_read(self, attempt=0):
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
			# self.hexel_ser = 'None'
			self.__set_ser('None')
			return 0

		# Create empty array to store values from memory addresses
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

			# Assign val to hexel serial
			# self.hexel_ser = val
			self.__set_ser(val)
			return val

		except:
			# print("WARNING: Failed to read I2C device for chan {}.".format(self.chan))
			# self.hexel_ser = 'None'
			self.__set_ser('None')

			return 0

		return 0

# ----------------------------------------------------------------------#


class Modules:
	"""Class that handles operations for all modules

	Attributes:
		modules (list): Holds all modules objects
		wd (float): Time that watchdog was triggered last
		max_time (float): Max time before watchdog shuts down lasers

	"""

	def __init__(self):
		"""initialize object"""
		self.modules = []  # Create empty list for modules
		self.wd = time.time()  # Watchdog Timer Start Time
		self.max_time = 30  # Max time before triggering turn-off
		self.intr_lck = False  # Interlock
		self.lds_connected = False  # Variable to indicated if lds are connected
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

		# number of checks
		checks = 0

		# Wait to see 3 usb devices that match Tom's boards
		while (seen_devices != DEVICES):

			# Seen if USB needs to be restarted
			if (checks > 30):
				usb_reset()

			else:
				# Loop delay
				time.sleep(1)

			# Reset seen_devices
			seen_devices = 0

			# Testing print statement
			if (TEST):
				print("Waiting to recognize all devices...")

			# List all ports
			ports = glob.glob('/dev/tty[A-Za-z]*')

			# List number of seen_devices
			for port in ports:
				if ('ttyACM' in port):
					seen_devices = seen_devices + 1

			# Increment number of checks
			checks = checks + 1

		# Give some time for devices to re-initialize
		print("Sleeping 15 seconds...")
		time.sleep(15)

		# Testing print statement
		if (TEST):
			print("Found 3 devices... Proceeding to connect.")

		# Connect to all ports
		for port in ports:
			try:
				if ('ttyACM' in port):
					# Instatiate Module Obkect
					module = Module()

					# Open serial port to laser driver
					module.open(port)

					# Find what channel is connected to what
					if (module.chan == "1"):
						module.button_pin = BUTTON_0  # Rasberry PI GPIO Pin
						module.i2c_addr = CHAN_ARRAY[0]  # I2C Mux Addr
					elif (module.chan == "2"):
						module.button_pin = BUTTON_1  # Rasberry PI GPIO Pin
						module.i2c_addr = CHAN_ARRAY[1]  # I2C Mux Addr
					elif (module.chan == "3"):
						module.button_pin = BUTTON_2  # Rasberry PI GPIO Pin
						module.i2c_addr = CHAN_ARRAY[2]  # I2C Mux Addr
					else:
						print("ERROR: Module channel listed as {}.".format(module.chan))
						break

					# Attach object to current current object in for of a list
					self.modules.append(module)

			# Exception for failing to connect to USB port
			except (OSError, serial.SerialException):
				print("ERROR: Could not connect to port {}".format(port))
				pass

		# Check length of module list
		if (len(self.modules) != 3):
			raise TomBoardError("TomBoardError: Only connected to {} out of boards.".format(len(self.modules)))

		# Sort the object list by laser driver channel
		self.modules.sort(key=lambda x: x.chan)

		# Indicate that laser drivers are all conncted
		self.lds_connected = True

		return None

	def update(self):
		"""Check button state"""
		# Check the watchdog timer
		if (time.time() - self.wd < self.max_time):

			# If the watchdog timer is satisfied, update modules
			for module in self.modules:
				module.update()

		else:
			# Turn off all devices if watchdog has not been reset
			if (TEST):
				print("WD Time Passed")

			# Turn all lasers off
			self.turn_all_off()

		return

	def clear_all(self):
		"""Clear all info from Tom's boards"""
		for module in self.modules:
			module.clear()
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

	def set_all_currents_list(self, curr_set_list: list):
		"""Set the current set point for all lasers"""
		for i in range(0, len(self.modules)):
			self.modules[i].curr_set = curr_set_list[i]
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
		if (len(self.modules) != 0):
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

		# Loop to establish ethernet port connection
		while self.eth_connected == False:
			try:
				# Bind socket
				self.s.bind((TCP_IP, TCP_PORT))
				self.s.listen(1)
				self.eth_connected = True

			# OS error exception catch
			except OSError as e:
				print(e)
				print("Failed to establish ethernet connection, trying again...")
				self.eth_connected = False
				time.sleep(1)

			# Generic exception catch
			except Exception as e:
				# Catch all
				print("Generic exception catch for ethernet connection.")
				print(e)
				try:
					# Close the socket
					self.s.close()

				except:
					pass

				return

		while True:
			try:
				# Create a waitable object with a timeout of 1 second
				readable, _, _ = select.select([self.s], [], [], 1.0)

				# Print for testing
				if TEST_2:
					print("Waiting for command")

				# Look at socket
				if self.s in readable:

					# Wait for connection
					self.conn, addr = self.s.accept()

					# Print for testing
					if (TEST_2):
						print('Connection address:', addr)

					# Recieve value from PC
					recv_val = self.conn.recv(BUFFER_SIZE).decode()

					# Check the recieved value
					if "ON" in recv_val:

						# Get the current setpoint
						try:
							curr_sets = recv_val.split(" ")
							curr0 = curr_sets[1]
							curr1 = curr_sets[2]
							curr2 = curr_sets[3]
							curr_list = [curr0, curr1, curr2]
						except:
							print("Invalid command")
							curr_list = ['0.1', '0.1', '0.1']

						# Set the software interlock on
						self.set_sw_intr_lck_on_all()

						# Set the current setpoints
						self.set_all_currents_list(curr_list)

					else:
						# Set the software interlock off
						self.set_sw_intr_lck_off_all()

						# Print for testing
						if TEST:
							print("SW interlock off")

					# Print for testing
					if TEST_2:
						print("received data:", recv_val)

					# Send out status update with latest data
					data = self.__str__().encode()  # Generate string
					self.conn.send(data)  # Send via TCP/IP

					# Set new watchdog start time
					self.wd = time.time()

			except Exception as e:
				# Close TCP/IP connection
				print("---------------------------------------")
				print(e)
				print("---------------------------------------")

				# Exit loop
				break

		# Close the socket
		try:
			self.s.close()

		# Generic exception catch
		except Exception:
			pass

		return

	def reset_usb(self) -> None:
		"""Turn on and off all USB connections and re-connect to laser drivers"""

		# Mark connected as false
		self.lds_connected = False

		# Try to close all connections w/o turning lasers off
		self.r_close_all()

		# Clear current and voltage variables for all modules
		self.clear_all()

		# Call bash script to restart all usb devices
		usb_reset()

		# Re-Connect to all the ports
		self.connect_ports()

		return None

	def close_all(self):
		"""Close all serial ports"""

		# Increment through all module objects
		for module in self.modules:

			# Close the module
			try:
				module.close()

			# Generic exception catch
			except:
				print("ERROR: Failed to close module {}".format(module.chan))

		return

	def r_close_all(self):
		"""Close all serial ports"""

		# Increment through all module objects
		for module in self.modules:

			# Close the module
			try:
				module.r_close()

			# Generic exception catch
			except:
				print("ERROR: Failed to close module {}".format(module.chan))

		return


# ----------------------------------------------------------------------#

class TomBoardError(Exception):
	"""Exception to handle errors relating to poor firmware on Tom's
	laser driver board"""
	pass


# ----------------------------------------------------------------------#

def main():
	"""Method to run the main program"""
	# Instantiate modules object
	ms = Modules()

	# Create thread for TCP/IP communication
	tcp_ip_thread = threading.Thread(target=ms.recv_and_send_loop)

	# Start thread
	tcp_ip_thread.start()

	try:
		# Update loop
		ms.connect_ports()

	# If error is found on Tom's board, loop and reset USB until no errors
	except TomBoardError:
		ms.lds_connected = False
		while (ms.lds_connected == False):
			try:
				ms.reset_usb()

			# Catch exception for Tom's board in the loop
			except TomBoardError as e:
				print(e)
				print("---------------------------------------------")
				pass

	# Block for the main update loop
	try:
		if (ms.lds_connected):
			# Set all currents to 0
			ms.set_all_currents('0')

			if (TEST_2):
				print("Enabling all interlocks.")
				ms.set_sw_intr_lck_on_all()

			# Loop to handle updating states of laser drivers
			while (True):

				# Try to updat
				try:
					ms.update()
					time.sleep(0.01)

				# If usb connection for Tom's board is found, enter reset loop
				except TomBoardError as e:
					ms.lds_connected = False
					while (ms.lds_connected == False):

						# Try to reset usb and reconnect to Tom's board
						try:
							ms.reset_usb()

							# Print for testing
							if (TEST):
								print("Enabling all interlocks.")
								ms.set_sw_intr_lck_on_all()

						# Catch exception for error on Tom's board in reset loop
						except TomBoardError as e:
							print(e)
							print("---------------------------------------------")
							pass

	# Keybaord interrupt for closing the program
	except KeyboardInterrupt:
		print("\n\nKeyboard interrupt triggered!\nClosing program.\n")

	# Generic exception check
	except Exception as e:
		print("Catch all excepting hit:")
		print(str(e))

	# Finally for on shutdown
	finally:

		# Shut down the socket
		ms.set_all_currents('0')

		# If the socket exists, close it
		if (ms.s != None):
			ms.s.close()

		# If the thread is alive, close it
		if (tcp_ip_thread.is_alive()):
			if (TEST):
				print("TCP/IP thread alive...")
				print("Joining thread...")

			# Join the thread
			tcp_ip_thread.join()

			# Print statement for testing
			if (TEST):
				print("Thread succesfully joined.")

	# Close all laser driver connections
	try:
		ms.close_all()
	except:
		pass

	# Cleanup GPIO pins
	try:
		GPIO.cleanup()
	except:
		pass


# ----------------------------------------------------------------------#

# Run the program
main()

# ----------------------------------------------------------------------#
