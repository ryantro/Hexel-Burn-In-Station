import serial

COM = r'/dev/ttyACM0'


try:
	ld = serial.Serial(COM, 115200, timeout = 3)
	print("Opening Serial Port...")
	
	cmd = "WHOAMI\r\n".encode()
	
	print("Writing command...")
	ld.write(cmd)
	
	print("Reading response...")
	resps = ld.read_until(">>>".encode()).decode()
	i = 0
	print(resps)
	lines = resps.split('\n')[1].split(' ')[3].split('\\')[0]
	print(lines)
	"""
	for resp in resps:
		print("line {}: ".format(i) + resp.decode())
		i = i + 1
	
	"""
	
	
	
finally:
	ld.close()
	pass
