
import glob
import serial



def port_assigner():

	ports = glob.glob('/dev/tty[A-Za-z]*')
	result = []
	for port in ports:
		try:
			s = serial.Serial(port)
			s.close()
			result.append(port)
		except (OSError, serial.SerialException):
			pass
	print(result)

port_assigner()
