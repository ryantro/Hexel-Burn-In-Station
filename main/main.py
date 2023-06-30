"""
Author: Ryan Robinson
Company: Nuburu
Project: Hexel Burn-In Station
"""

__version__ = "0.1"
__author__ = "Ryan Robinson"
__email__ = "ryan.robinson@nuburu.net"
__status__ = "Prototype"

import time
import threading
import socket
from contextlib import contextmanager
from rich import box
from rich.align import Align
from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.text import Text

# TESTING IMPORTS
import random

#----------------------------------------------------------------------------------#

l1=r"      ___           ___           ___           ___           ___           ___     "
l2=r"     /\__\         /\__\         /\  \         /\__\         /\  \         /\__\    "
l3=r"    /::|  |       /:/  /        /::\  \       /:/  /        /::\  \       /:/  /    "
l4=r"   /:|:|  |      /:/  /        /:/\:\  \     /:/  /        /:/\:\  \     /:/  /     "
l5=r"  /:/|:|  |__   /:/  /  ___   /::\~\:\__\   /:/  /  ___   /::\~\:\  \   /:/  /  ___ "
l6=r" /:/ |:| /\__\ /:/__/  /\__\ /:/\:\ \:|__| /:/__/  /\__\ /:/\:\ \:\__\ /:/__/  /\__\ "
l7=r" \/__|:|/:/  / \:\  \ /:/  / \:\~\:\/:/  / \:\  \ /:/  / \/_|::\/:/  / \:\  \ /:/  /"
l8=r"     |:/:/  /   \:\  /:/  /   \:\ \::/  /   \:\  /:/  /     |:|::/  /   \:\  /:/  / "
l9=r"     |::/  /     \:\/:/  /     \:\/:/  /     \:\/:/  /      |:|\/__/     \:\/:/  /  "
la=r"     /:/  /       \::/  /       \::/__/       \::/  /       |:|  |        \::/  /   "
lb=r"     \/__/         \/__/         ~~            \/__/         \|__|         \/__/	 "

lines = [l1,l2,l3,l4,l5,l6,l7,l8,l9,la,lb]
mstr = "\n".join(lines)
text = Text(mstr)
text.stylize("bold cyan")

m0=r"  _    _               _   ____                       _____          _____ _        _   _             "
m1=r" | |  | |             | | |  _ \                     |_   _|        / ____| |      | | (_)            "
m2=r" | |__| | _____  _____| | | |_) |_   _ _ __ _ __ ______| |  _ __   | (___ | |_ __ _| |_ _  ___  _ __  "
m3=r" |  __  |/ _ \ \/ / _ | | |  _ <| | | | '__| '_ |______| | | '_ \   \___ \| __/ _` | __| |/ _ \| '_ \ "
m4=r" | |  | |  __/>  |  __| | | |_) | |_| | |  | | | |    _| |_| | | |  ____) | || (_| | |_| | (_) | | | |"
m5=r" |_|  |_|\___/_/\_\___|_| |____/ \__,_|_|  |_| |_|   |_____|_| |_| |_____/ \__\__,_|\__|_|\___/|_| |_|"
                                                                                                      
lines2 = [m0,m1,m2,m3,m4,m5]
mstr2 = "\n".join(lines2)
text2 = Text(mstr2)
text2.stylize("bold cyan")

# Hexel Burn-in Station Dimensions
COLUMNS = 7
ROWS = 4

#----------------------------------------------------------------------------------#

class HexelData:
	def __init__(self):
		# To report
		self.serNum = None		# Serial number
		self.status = 'No device' # Device status [No Device, Off, On]
		self.power = 0			# Optical power
		self.cwl = 0			# Center wavelength
		self.volt = 0			# Voltage drop
		self.cur = 0 			# Device current
		self.run_time = 0		# Burn in run time
		self.slopeShort = None		# Slope in the last 10 hours

		# Internal
		self.row = None
		self.column = None
		self.__style = "black"
		self.__len_limit = 99999 # maximum length of list
		self.__powers = []
		self.__times = []
		self.__measure_num = 0
		self.__recent_time = time.time()
		self.__start_time = time.time()
		self.__start_power = 0
		self.__start_cwl = 0

		self.com_fail = False

		# Time to store data
		self.__store_time_lim = 5 * 3600 # max data storage set to 5 hours 

		return None

	def pm_osa_update(self, power, cwl) -> None:
		"""Update the object with the data from the OSA and power meter."""
		
		try:
			# Update measurement number by 1
			self.__measure_num = self.__measure_num + 1

			# Update power with most recent measurement
			self.power = float(power)

			# Update central wavelength with most recent measurement
			self.cwl = float(cwl)

			# Update recent time
			self.__recent_time = time.time()

			# Update run time
			self.run_time = self.__recent_time - self.__start_time

			# Call power append method
			self.__data_append()

			#########################################
			# TODO: Calculate long term degredation #
			#########################################

		except:
			# If it fails to store any data as floats
			self.status = "ERROR: pm/osa update"

		return

	def __data_append(self) -> None:
		"""Append most recent power measurement to power list"""

		# Find storage time length
		if(len(self.__times) >= 2):
			storage_time = self.__times[-1] - self.__times[0]
		
		# Empty list case
		else:
			storage_time = 0

		# Check if storage time has been reached
		if(self.__store_time_lim > storage_time):
			self.__powers.append(self.power)
			self.__times.append(self.__recent_time - self.__start_time)

		# List limit has been reached
		else:
			try:
				self.__powers = self.__powers[1:] + [self.power]
				self.__times = self.__times[1:] + [self.__recent_time - self.__start_time]
			except:
				self.status = "ERROR: list update"

		# Calculate short term degredation 
		if(len(self.__times) >= 2):

			# Find time and power means
			t_bar = sum(self.__times) / len(self.__times)
			p_bar = sum(self.__powers) / len(self.__powers)

			# Numerator and Divisor
			top = 0
			bottom = 0

			for i in range(0,len(self.__times)):
				top = top + (self.__times[i] - t_bar) * (self.__powers[i] - p_bar)
				bottom = bottom + ((self.__times[i] - t_bar)**2)

			# Calculate short term slope
			if(bottom != 0):
				self.slopeShort = top/bottom

			else:
				self.slopeShort = 0

		##########################################
		# TODO: Calculate short time degredation #
		##########################################

		return

	def __str__(self) -> str:
		"""Generate a text object"""

		if(self.com_fail == False):
			# Generate Text object for return string
			rlist = ['', '', '', '', '', '', '', '', '', '']
			rlist[0] = "[{}]".format(self.serNum)
			rlist[1] = "|--Status: {}".format(self.status)
			rlist[2] = "|----Burn-In Time: {:.0f} (hr)".format(self.run_time)
			rlist[3] = "|----Voltage Drop: {:.2f} (V)".format(self.volt)		
			rlist[4] = "|----Set Current: {:.2f} (A)".format(self.cur)		
			rlist[5] = "|--Power: {:.2f} (W)".format(self.power)
			rlist[6] = "|----Change: {:.2f} (W)".format(self.power - self.__start_power)
			
			try:
				rlist[7] = "|----Degredation: {:.2f} (%/khr)".format(self.slopeShort)
			except:
				rlist[7] = "|----Degredation: {}".format(self.slopeShort)
			
			rlist[8] = "|--Wavelength: {:.2f} (nm)".format(self.cwl)
			rlist[9] = "|----Shift: {:.2f} (nm)".format(self.cwl - self.__start_cwl)

			# Create return string
			retString = '\n'.join(rlist)
			retString = Text(retString)
			retString.stylize(self.__style)

		else:
			# Generate ascii art for com failure
			rlist = ['', '', '', '', '', '', '', '', '', '']
			rlist[0] = "[COM FAILURE]"
			rlist[1] = "|-------XXX------------XXX-------|"
			rlist[2] = "|---------XXX--------XXX---------|"
			rlist[3] = "|-----------XXX----XXX-----------|"
			rlist[4] = "|-------------XXXXXX-------------|"
			rlist[5] = "|---------------XX---------------|"
			rlist[6] = "|-------------XXXXXX-------------|"
			rlist[7] = "|-----------XXX----XXX-----------|"
			rlist[8] = "|---------XXX--------XXX---------|"
			rlist[9] = "|-------XXX------------XXX-------|"

			# Create return string
			retString = '\n'.join(rlist)
			retString = Text(retString)
			self.set_red()
			retString.stylize(self.__style)

		return retString

	def com_fail(self) -> None:
		"""Method to indicate communication failure with rasberry pi"""

		# Show com failure
		self.__com_fail = True

		# Clear object
		self.__clear()

		# Set status to show error
		self.status = "ERROR: RP Com"

		# Turn text red
		self.set_red()

		return None

	def rp_update(self, serNum: int, status: str, volt: float, cur: float) -> None:
		"""Update the object with the data from the Rasberry Pi
		- serNum can be either 'None' or an integer like '2001235'
		- status can be either 'on' or 'off'
		- volt is a float, e.g. 29.56
		"""

		# Check for status change
		self.__check_status_change(status)

		# If no module is in the spot
		if(serNum == None):
			self.__clear()

		# If updating the same hexel
		elif(self.serNum == serNum):
			
			# Update voltage
			try:
				self.volt = float(volt)
				self.cur = float(cur)
			except:
				self.volt = "ERROR"
				self.cur = "ERROR"

		# If updating a new hexel
		else:
			# New serial number placed on station
			self.serNum = serNum
			
			# Update voltage
			try:
				self.volt = float(volt)
				self.cur = float(cur)
			except:
				self.volt = "ERROR"
				self.cur = "ERROR"

		return None

	def __check_status_change(self, status: str) -> None:
		"""Update the status of the module and marks start time"""

		status = status.replace(' ','')

		if(self.status == 'Off' or self.status == 'No device'):

			if(status == 'On'):
				self.__start_time = time.time() # Marking start time
				self.status = 'On' # Marking device turning On
				self.__style = "bold cyan"

			elif(status == 'Off'):
				self.status = 'Off'
				self.__style = "white"

			else:
				self.status = 'No device'
				self.__style = "cyan"

		elif(self.status == 'On' or self.status == 'No device'):

			if(status == 'Off'):
				self.status = 'Off' # Marking device turning Off
				self.__style = "white"

			elif(status == 'On'):
				self.status = 'On'
				self.__style = "bold cyan"

			else:
				self.status = 'No device'
				self.__style = "cyan"

		return

	def __clear(self) -> None:
		"""Reset all internal variables"""

		# Clear public variables
		self.serNum = None		# Serial number
		self.status = 'No device'	# Device status [No Device, Off, On]
		self.power = 0			# Optical power
		self.cwl = 0			# Center wavelength
		self.volt = 0			# Voltage drop
		self.run_time = 0		# Burn in run time
		self.slopeShort = None 	# Slope in the last 10 hours

		# Clear private variables
		self.__times = []
		self.__powers = []
		self.__measure_num = 0
		self.__recent_time = time.time()
		self.__start_time = time.time()
		self.__start_power = 0
		self.__start_cwl = 0


		return None

	def set_color(self, i) -> None:
		"""Set the text based"""
		if(i > 255):
			r = 255 - (i - 255)
			g = i - 255
		else:
			g = i
			r = 255
		self.__style = "bold rgb({},{},0)".format(r,g)
		return None

	def set_white(self) -> None:
		"""Sets the text to white"""
		self.__style = "white"
		return None

	def set_cyan(self) -> None:
		"""Sets the text to cyan"""
		self.__style = "cyan"
		return None

	def set_red(self) -> None:
		"""Sets the text to red"""
		self.__style = "bold rgb(255,0,0)"
		return None

	def set_yellow(self) -> None:
		"""Sets the text to red"""
		self.__style = "rgb(255,51,51)"
		return None

	def set_green(self) -> None:
		"""Sets the text to green"""
		self.__style = "bold green"
		return None

#----------------------------------------------------------------------------------#

class RpLookupTable:
	"""Stores IP address of all rasberry Pi's"""

	RP0_IP = '192.168.0.169' # IP Address
	RP0_C = [1, 2, 3]		 # Channels
	RP0_M = [0, 1, 2]		 # Module positions
	RP0 = [RP0_IP, RP0_C, RP0_M]

	RP1_IP = '192.168.0.170'
	RP1_C = [1, 2, 3]
	RP1_M = [3, 4, 5]
	RP1 = [RP1_IP, RP1_C, RP1_M]

	RP2_IP = '192.168.0.171'
	RP2_C = [1, 2, 3]
	RP2_M = [6, 7, 8]
	RP2 = [RP2_IP, RP2_C, RP2_M]

	RP3_IP = '192.168.0.172'
	RP3_C = [1, 2, 3]
	RP3_M = [9, 10, 11]
	RP3 = [RP3_IP, RP3_C, RP3_M]

	RP4_IP = '192.168.0.173'
	RP4_C = [1, 2, 3]
	RP4_M = [12, 13, 14]
	RP4 = [RP4_IP, RP4_C, RP4_M]

	RP5_IP = '192.168.0.174'
	RP5_C = [1, 2, 3]
	RP5_M = [15, 16, 17]
	RP5 = [RP5_IP, RP5_C, RP5_M]

	RP6_IP = '192.168.0.175'
	RP6_C = [1, 2, 3]
	RP6_M = [18, 19, 20]
	RP6 = [RP6_IP, RP6_C, RP6_M]

	RP7_IP = '192.168.0.176'
	RP7_C = [1, 2, 3]
	RP7_M = [21, 22, 23]
	RP7 = [RP7_IP, RP7_C, RP7_M]

	RP8_IP = '192.168.0.177'
	RP8_C = [1, 2, 3]
	RP8_M = [24, 25, 26]
	RP8 = [RP8_IP, RP8_C, RP8_M]

	RP9_IP = '192.168.0.178'
	RP9_C = [1]
	RP9_M = [27]
	RP9 = [RP9_IP, RP9_C, RP9_M]

	RP_LIST = [RP0, RP1, RP2, RP3, RP4, RP5, RP6, RP7, RP8, RP9]

	def get_mod_pos(self, rp_num: int, chan: int) -> int:
		"""Get the module position for a specific channel"""

		# Isolate to specific Rasberry Pi
		rp = self.RP_LIST[rp_num]
		chans = rp[1]
		pos = -1

		# Find corresponding channel
		for i in range(0,len(chans)):
			if(chans[i] == chan):
				pos = rp[2][i]
				break

		return pos

#----------------------------------------------------------------------------------#

class DataTable:
	"""Holds data for 28 module in hexelDataObjList
	Generates and updates table objects
	"""

	def __init__(self) -> None:
		# Create console object
		self.console = Console()

		# Clear the console
		self.console.clear()

		# Table object
		self.table = Table(show_lines = True)

		# Generate list for data objects
		self.hexelDataObjList = []

		# Generate hexel data objects to store serial number, power, cwl, degredation, etc..
		for i in range(0,ROWS):
			for j in range(0,COLUMNS):
				hexelData = HexelData()
				
				# Set row and column info
				hexelData.row = i
				hexelData.column = j

				# Append object to list
				self.hexelDataObjList.append(hexelData)

		# Create threading event
		self.event = threading.Event()
		self.event.clear() # Ensure it will return false

		# Create thread
		self.__table_thread = threading.Thread(target = self.__table_update_loop)

		return None

	def __gen_row_str(self, row: int) -> list:
		"""Generate string to create a single table row"""

		row_str = []
		row_str.append("[b]Row {}[/b]".format(row+1))

		for hexelDataObj in self.hexelDataObjList:
			if(hexelDataObj.row == row):
				row_str.append(hexelDataObj.__str__())

		return row_str

	def __generate_table(self) -> Table:
		"""Generate table object"""
		self.table = Table(box = box.ASCII_DOUBLE_HEAD, show_lines = True) #show_lines = True)

		# Create column headers
		self.table.add_column("Row", style="white", footer_style = "bold")
		for i in range(0,COLUMNS):
			self.table.add_column("Column {}".format(i+1), ratio = 1)

		# Fill out table rows
		for i in range(0,ROWS):
			hexels = self.__gen_row_str(i) # Create row
			self.table.add_row(*hexels)

		# Set table width to console width
		self.table.width = self.console.width

		return self.table

	def print_header(self) -> None:
		"""Print the station title header"""
		self.console.print(text)
		self.console.print(text2)

		return

	def start_update_loop(self) -> None:
		"""Method to create and run tableThread"""

		# Ensure thread isn't running
		if(self.__table_thread.is_alive() == False):

			# Ensure event is cleared
			self.event.clear()

			# Create thread
			self.__table_thread = threading.Thread(target = self.__table_update_loop)

			# Start the thread
			self.__table_thread.start()

		else:

			print("ERROR in starting thread is still alive!")

		return

	def stop_update_loop(self) -> None:
		"""Method to stop and join tableThread"""

		if(self.__table_thread.is_alive() == True):

			# Set the event to close the thread
			self.event.set()

			# Join the thread
			self.__table_thread.join()

		else:

			self.console.print("ERROR in closing thread, thread is already dead")

		return

	def __table_update_loop(self) -> None:
		"""Method that runs the forever loop"""

		###############################
		# TODO: Finalize this section #
		###############################

		with Live(console=self.console, screen=False, refresh_per_second=20) as live:

			while(self.event.is_set() == False):

				time.sleep(0.10)

				if(False):
					y = random.randint(0,27)
					self.hexelDataObjList[y].serNum = random.randint(2000000, 2999999)
					self.hexelDataObjList[y].set_white()

				if(False):
					y = random.randint(0,27)
					self.hexelDataObjList[y].serNum = random.randint(2000000, 2999999)
					self.hexelDataObjList[y].set_red()

				if(False):
					y = random.randint(0,27)
					self.hexelDataObjList[y].serNum = random.randint(2000000, 2999999)
					self.hexelDataObjList[y].set_cyan()

				if(False):
					y = random.randint(0,27)
					self.hexelDataObjList[y].serNum = random.randint(2000000, 2999999)
					self.hexelDataObjList[y].set_yellow()

				if(False):
					y = random.randint(0,27)
					self.hexelDataObjList[y].serNum = random.randint(2000000, 2999999)
					self.hexelDataObjList[y].set_green()

				if(False):
					y = random.randint(0,27)
					g = random.randint(0,255+255)
					self.hexelDataObjList[y].serNum = random.randint(2000000, 2999999)
					self.hexelDataObjList[y].set_color(g)

				live.update(self.__generate_table())

		return

#----------------------------------------------------------------------------------#

class R_Pi_Com:
	"""Class to handle communicating with all Rasberry Pi's via ethernet"""
	
	TCP_PORT = 9999
	BUFFER_SIZE = 1024

	def __init__(self) -> None:

		# Create threading event
		self.event = threading.Event()
		self.event.clear() # Ensure it will return false

		# Create thread
		self.__table_thread = threading.Thread(target = self.__rp_loop)

		# Data table placeholder
		self.dataTable = None

		# Object that contains all info about:
		# 	- IP Addr
		#	- Channels
		#	- Module Position
		self.rp_info = RpLookupTable()

		return

	def parse_and_update(self, i: int, dataTable: DataTable, chan_states: str) -> None:
		"""Read status from Rasberry Pi
		Inputs:
			- i (int): Rasberry Pi number, can be 0 throught 9
			- dataTable (DataTable): Data table object to update
			- chan_states (str): String recieved from Rasberry Pi
		"""

		# Select specific Rasberry Pi
		rp = self.rp_info.RP_LIST[i]

		chan_states_list = chan_states.split('\n')

		# Look through all channels and parse info
		for chan_state_l in chan_states_list:
			 chan_state = chan_state_l.split(',')

			 # Get laser driver channel and convert to int
			 chan = int(chan_state[0])		# Laser driver channel
			 status = str(chan_state[1])	# Channel status
			 
			 # Get hexel serial number
			 h_ser = chan_state[2].replace(' ','')	# Hexel serial number
			 
			 # Rasberry Pi communication failure flag
			 com_fail = False

			 # Parse h_ser
			 if('None' in h_ser):
			 	h_ser = None				# No hexel in position
			 
			 elif(h_ser == 'COM_FAIL'):
			 	com_fail = True
			 	
			 else:
			 	pass						# Hexel exists in position
			 
			 # Get voltage and current and convert string to float
			 volt = float(chan_state[3])	# Hexel voltage drop
			 cur = float(chan_state[4])		# Hexel set current

			 # Get module positon
			 mod_index = self.rp_info.get_mod_pos(i, chan)

			 # Check if position is used
			 if(mod_index != -1):

			 	# Update data list object
			 	dataTable.hexelDataObjList[mod_index].rp_update(h_ser, status, volt, cur)
			 	dataTable.hexelDataObjList[mod_index].com_fail = com_fail

		return None

	def start_rp_loop(self, dataTable: DataTable) -> None:
		"""Start the loop to continually read from the Rasberry Pi's
		Does the following:
			- Checks ethernet com w/ rasberry pi board
			- For channels 1-3, get:
				- Button state
				- Serial number
				- Voltage drop
				- Current
		"""

		# Attach dataTable object
		self.dataTable = dataTable

		# Start the thread if it isn't alive
		if(self.__table_thread.is_alive() == False):
			
			# Clear the event and start the thread
			self.event.clear()
			self.__table_thread = threading.Thread(target = self.__rp_loop)
			self.__table_thread.start()

		return

	def __rp_loop(self) -> None:
		"""Loop to continually read from the Rasberry Pi's"""

		while(self.event.is_set() == False):

			time.sleep(0.25)

			# Loop through all Rasberry Pi's
			for i, rp in enumerate(self.rp_info.RP_LIST):

				# Get Rasberry Pi Ip Address
				rp_ip = rp[0] # Rasberry Pi IP

				#######################################
				# TODO: Method to communicate over IP #
				#######################################

				L1 = "1, On, 2001712, 30, 3.5"
				L2 = "2, Off, 2002366, 30, 0"
				L3 = "3, No device, None, 30, 0"
				EXAMPLE_STATES = '\n'.join([L1, L2, L3])

				# Code to communicate via TCP IP
				try:
					TCP_PORT = 9999
					sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create socket
					sock.settimeout(0.5) 			# Set timeout to 0.5 seconds
					sock.connect((rp_ip, TCP_PORT))	# Connect
					sock.send(bytes("ON", 'utf-8')) # Send message
					data = sock.recv(1024).decode() # Recieve data
					sock.close()					# Close socket
				
				# Set communication failure status
				except:
					l1 = "1, No device, COM_FAIL, 0, 0"
					l2 = "2, No device, COM_FAIL, 0, 0"
					l3 = "3, No device, COM_FAIL, 0, 0"
					data = '\n'.join([l1, l2, l3])
				
				# Update table with info
				self.parse_and_update(i, self.dataTable, data)

		return

	def stop_rp_loop(self) -> None:
		"""Stop the loop continually reading from the Rasberry Pi's"""

		# Join the thread if it is alive
		if(self.__table_thread.is_alive() == True):
			
			# Set the event and join the thread
			self.event.set()
			self.__table_thread.join()

		return

#----------------------------------------------------------------------------------#

class Application:
	def __init__(self):
		return

	def run(self):

		# Test string
		HEXEL = "[b][20001234][/b]\n  Status: Off\n  Power: 30 W\n  Center Wl: 438 nm\n  Voltage: 29 V\n  Time: 48 hr"

		# Hexel list
		hexels = []
		hexels.append("Row 1")
		for i in range(0,COLUMNS):
			hexels.append(HEXEL)

		# Create objects
		console = Console()
		console.clear()

		DT = DataTable()
		table = DT.generate_table()

		# table = Table(show_lines = True)
		table_centered = Align.center(table)

		# Clear the console
		console.clear()

		# Print the Nuburu: Hexel Burn-In Station header
		console.print(text)
		console.print(text2)

		# Find max console width
		width = console.measure(table).maximum

		with Live(table_centered, console=console, screen=False, refresh_per_second=20) as live:
	
			time.sleep(1)

			table.width = console.width

			time.sleep(1)

			DT.hexelDataObjList[0].serNum = 20001234

			live.update(DT.generate_table(console.width))

		return

#----------------------------------------------------------------------------------#

def main():

	# Create data table object
	table = DataTable()

	# Print main header
	table.print_header()

	# Start thread
	table.start_update_loop()

	# Create Rasberry Pi communications object
	
	rpc = R_Pi_Com()
	
	rpc.start_rp_loop(table)


	# Start time
	t1 = time.time()

	while(time.time() - t1 < 60):
		pass

		# EXAMPLE_STATES = '1, On, 2001712, 0, 0\n2, Off, None, 0, 0\n3, Off, None, 0, 0'
		# rpc.parse_and_update(1, table, EXAMPLE_STATES)

		# Get update from Rasberry Pi's
		# for pi in pis:
		# 	A = Get_Update(pi)
		#   app.rp_table_update(A) # feed updated to DataTable object

	# Stop thread

	rpc.stop_rp_loop()

	table.stop_update_loop()

	print("DONE!")

#----------------------------------------------------------------------------------#

if __name__ == '__main__':
	main()

#----------------------------------------------------------------------------------#