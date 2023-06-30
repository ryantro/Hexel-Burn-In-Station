"""
Author: Ryan Robinson
Company: Nuburu
Project: Hexel Burn-In Station
"""

import socket

TCP_IP = '192.168.0.169'
TCP_PORT = 9999
BUFFER_SIZE = 1024
MESSAGE = bytes("ON", 'utf-8')
host = socket.gethostname()
print("HOST: " + host)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((TCP_IP, TCP_PORT))
s.send(MESSAGE)
data = s.recv(BUFFER_SIZE)
s.close()

print("received data:")
print(data.decode())