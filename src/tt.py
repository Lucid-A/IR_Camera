import serial

ser = serial.Serial(r"/dev/serial0", 9600, timeout=0.5)

if not ser.isOpen():
    ser.open()

# set query mode
cmd = b"\xA4\x06\x03\x01\xAE"
ser.write(cmd)
data = ser.read(6)
print(data)

# read registers 0B..4A
cmd = b"\xA4\x03\x0B\x40\xF2"
ser.write(cmd)
data = ser.read(4+64+1)
print(list(data[4:-1]))

'''
for i in range(5):
    frame = ser.read(64)
    print(frame)
    print()
'''

ser.close()

