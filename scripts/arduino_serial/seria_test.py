import serial

ser = serial.Serial('/dev/ttyACM4',9600)
s = [0,1]
while True:
	read_serial = ser.readline()
	#LR = read_serial.split(",")
	#print LR[0]
	print read_serial

	#s[0] = str(int (ser.readline(),16))
	#print s[0]