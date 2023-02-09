from IR_sensor import IR_sensor_reg, IR_sensor

sensor = IR_sensor()
sensor.close()
sensor.open()
sensor.write_reg(IR_sensor_reg.output_mode, 1)
print(sensor.read_reg(reg_start=0xB, num_regs=64))

'''
import serial

ser = serial.Serial(r"/dev/serial0", 9600, timeout=0.5)

if not ser.isOpen():
    ser.open()

for i in range(5):
    frame = ser.read(64)
    print(frame)
    print()

ser.close()
'''
