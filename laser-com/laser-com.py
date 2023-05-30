"""
AUTHOR: RYAN ROBINSON

FUCTION:
	COMMUNICATE WITH LASER DRIVERS.

"""

import glob
import serial

class Laser:
	def __init__(self):
		self.addr = None
		self.ser = None
		self.connected = False
		self.chan = None
		self.hexel = None # Hexel serial number
	
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
	
	# SHOULD THIS BE A PART OF THIS OBJECT? OR SEPERATE?
	def findSer(self):
		return
	
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

def port_finder():
	ports = glob.glob('/dev/tty[A-Za-z]*')
	result = []
	for port in ports:
		try:
			#s = serial.Serial(port)
			#s.close()
			if('ttyACM' in port):
				result.append(port)
		except (OSError, serial.SerialException):
			pass
	print(result)
	return result

ports = port_finder()

LS = []

for port in ports:
	try:
		L = Laser()
		print("Opening {}...".format(port))
		L.open(port)
		if(L.connected == True):
			LS.append(L)
			L.getVoltage()
		
	finally:
		L.close()
		
