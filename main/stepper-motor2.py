import socket



HEADER = bytes([0x00, 0x07])
END = bytes([0xD])
cmd = "FP0" # 99999"
enc_cmd = cmd.encode()
to_send = HEADER + enc_cmd + END	# Add header and end bits

TCP_PORT = 7776
stm_ip = '192.168.0.80'
try:
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create socket
	sock.settimeout(0.5) 			# Set timeout to 0.5 seconds
	sock.connect((stm_ip, TCP_PORT))	# Connect
	sock.send(to_send) # Send message
	data = sock.recv(1024).decode() # Recieve data
	print(data)
except Exception as e:
	print(e)
finally:
	sock.close()