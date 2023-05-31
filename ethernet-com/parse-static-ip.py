with open('/etc/dhcpcd.conf') as fin :
    lines = fin.readlines()

line_num = -1
for k,line in enumerate(lines) :
	# print(line)
	if "static ip_address" in line:
		line_num = k

a = lines[line_num].split("=")[-1].strip(" ").strip("/n")
print(a)
