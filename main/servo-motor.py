"""
Author: Ryan Robinson
Company: Nuburu
Project: Hexel Burn-In Station
File: stepper-motor.py

Notes:
	Code written to communicate with an Applied-Motion STM23IP-3EN stepper motor.
	Functions to control the horizontal stage for the Hexel Burn-In Station.
	Motor IP is set to 192.168.0.80 via the dial on the motor. 
"""

__version__ = "0.1"
__author__ = "Ryan Robinson"

import socket
import netifaces
import time

class StepperError(Exception):
	pass

class Stepper_Motor:
	"""Class to control the stepper motor for the hexel burn in station"""

	TIMEOUT = 60*1 # Timeout time set to 1 minute

	__TEST = True # print statements for testing
	__TCP_PORT = 7776		# Define by motor
	__STM_IP = '192.168.0.110' # This must be configured via dial on front
	__HEADER = bytes([0x00, 0x07])
	__END = bytes([0xD])

	def __init__(self, sock = None) -> None:
		"""Initialize object.
		
		Input:
			- sock: socket object
		"""

		# Command times and minimum time
		self.__last_cmd = 0 	# Time that last command was sent
		self.__min_time = 0.1 			# Minimum time between commands

		# Create new socket if none exists
		if sock is None:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			print("created socket!")
		else:
			self.sock = sock
		
		 # Create socket
		self.sock.settimeout(1.0) 							# Set timeout to 0.5 seconds
		self.sock.connect((self.__STM_IP, self.__TCP_PORT))	# Connect

		self.__set_presets()

		return None

	def __set_presets(self) -> None:

		cmd_mode = "CM21"
		self.__send(cmd_mode)

		lmt_set = "DL1"
		self.__send(lmt_set)

		vel_set = 'VE5'
		self.__send(vel_set)

		acc_set = ''

		dec_sec = ''

		pass

	def __send(self, cmd: str) -> str:
		"""Send a command to the stepper motor
		Full Command list:
			- https://appliedmotion.s3.amazonaws.com/Host-Command-Reference_920-0002W_0.pdf

		Inputs:
			- cmd: command
		
		Outputs:
			- ret_str: return string
		"""

		# Format and send command
		enc_cmd = cmd.encode()
		to_send = self.__HEADER + enc_cmd + self.__END	# Add header and end bits

		# Wait for time between commands
		while(time.time() - self.__last_cmd < self.__min_time):
			time.sleep(0.2)
		
		# Send message to stepper motor
		if(self.__TEST): print("Sent Command: {}".format(cmd))
		self.sock.send(to_send) 

		# Wait 50 ms
		time.sleep(0.2)

		# Get return val
		try:
			ret_str = self.sock.recv(1024).decode()
			if(self.__TEST): print("Recieved String: {}".format(ret_str))
		except:
			return "ERROR: Timeout Reached"

		return ret_str

	def __wait_for_move(self) -> None:
		"""Wait for stage movement to finish"""

		cmd = "SC" # command to get status
		start_time = time.time()
		status = ''
		status_list = ['0001', '0201', '8009']

		while(not any(s in status for s in status_list)):
			time.sleep(1)
			status = self.__send(cmd)

			if(time.time() - start_time > self.TIMEOUT):
				if(self.__TEST): print("ERROR: Stage took to long to move!")
				stop_cmd = "ST"
				self.__send(stop_cmd)
				raise StepperError("Stage move alloted time limit hit")

		if(self.__TEST): print("Move finished!")

		return None


	def __enter__(self, sock = None):
		"""__enter__ method for the with command"""
		self.__init__(sock = sock)
		return self

	def __exit__(self) -> None:
		"""__exit__ method for the with commnd"""
		self.close()
		return None

	def home(self):
		"""Method to home the seek home"""		

		# Send command to home motor
		home_cmd = "SH1H"
		self.__send(home_cmd)

		# Wait for homing to finish
		self.__wait_for_move()

		# Set encoder position to zero
		enc_zero = "EP0"
		self.__send(enc_zero)

		# Set position to zero
		sp_zero = "SP0"
		self.__send(sp_zero)

		return self

	def stop(self) -> None:
		self.__send("ST")
		return None

	def test(self) -> None:
		"""Test move the motor
		inputs:
			- steps: int or str
		"""

		cmd = "FL-999999"
		print(cmd)
		ret = self.__send(cmd)
		print(ret)
		self.__wait_for_move()

		return None

	def close(self):
		try:
			self.sock.close()
			if(self.__TEST): print("closed!")
		except Exception as e:
				print(e)

	


print("test")
try:
	a = Stepper_Motor()
	a.home()
	#a.test()
finally:
	a.close()