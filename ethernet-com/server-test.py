import socket

TCP_IP = '192.168.0.169' #'192.168.0.255' # this IP of my pc. When I want raspberry pi 2`s as a server, I replace it with its IP '169.254.54.195'
TCP_PORT = 9999
BUFFER_SIZE = 1024
#TCP_IP = '0.0.0.0'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)


while 1:
	conn, addr = s.accept()
	print ('Connection address:', addr)
	print("Waiting for data...")
	data = conn.recv(BUFFER_SIZE)
	print("received data:", data)
	conn.send(data)  # echo
conn.close()
