# declaring an integer value
integer_val = 99999999
print("Start val")
print(integer_val)  

bytes_val = integer_val.to_bytes(4, 'big')
  
# printing integer in byte representation
print(bytes_val)

info = [bytes_val[i:i + 1] for i in range(0, len(bytes_val), 1)]
print("byte array")
print(info)

print("joined")
b = b''.join(info)
print(b)

print("Converted back to int")  
int_val = int.from_bytes(b, "big")
print(int_val)

