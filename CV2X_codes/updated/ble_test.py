import time
serial_port_path = '/dev/ttymxc3'
baud_rate = 115200

# Open the serial port as a file
ser_file = open(serial_port_path, 'wb+', buffering=0)

#reset the connection
ser_file.write("AT+CPWROFF\r\n".encode())
time.sleep(5)

# Optional: Set the baud rate (if supported by the device)
ser_file.write("ATE\r\n".encode())

# create service ID
ser_file.write("AT+UBTGSER=4906276bda6a4a6cbf9473c61b96433c\r\n".encode())

# create characteristic ID
ser_file.write("AT+UBTGCHA=49af5250f17646c5b99aa163a672c042,12,1,1,00,1,1\r\n".encode())

#time.sleep(5)



#write your alerts below:


#ser_file.write("AT+CPWROFF\r\n".encode())
ser_file.close()
