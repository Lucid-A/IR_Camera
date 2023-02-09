from enum import IntEnum
from serial import Serial
import logging as log

log.basicConfig(
        level=log.DEBUG,
#       filename='IR_sensor.log',
#       filemode='w'
        )


class IR_sensor_Reg(IntEnum):

    addr_ID         = 0x00
    baud_rate       = 0x01
    update_hz       = 0x02
    output_mode     = 0x03
    img_output_fmt  = 0x04
    config          = 0x05
    target_temp_offset_compsente = 0x06
    emissivity      = 0x07
    threshold       = 0x08
    ta_h            = 0x09
    ta_l            = 0x0A
    to_h8_start     = 0x0B
    to_h8_end       = 0x4A
    to_l8_start     = 0x4B
    to_l8_end       = 0x8A
    to_over_threshold_start = 0x8B
    to_over_threshold_end   = 0x92
    max_temp_h8     = 0x93
    max_temp_l8     = 0x94
    min_temp_h8     = 0x95
    min_temp_l8     = 0x96
    firmware        = 0x97

class IR_sensor_Baud_rate(IntEnum):
    baud_2400   = 0x00
    baud_4800   = 0x01
    baud_9600   = 0x02
    baud_19200  = 0x03
    baud_38400  = 0x04
    baud_57600  = 0x05
    baud_115200 = 0x06
    baud_230400 = 0x07

class IR_sensor_Output_mode(IntEnum):
    continuous   = 0x00
    query        = 0x01
    auto_char    = 0x02

class IR_sensor_Img_transform(IntEnum):
    origin = 0x00
    rotate_clockwise_90     = 0x01
    rotate_clockwise_180    = 0x02
    rotate_clockwise_270    = 0x03
    mirror  = 0x04
    rotate_clockwise_90_mirror  = 0x05
    rotate_clockwise_180_mirror = 0x06
    rotate_clockwise_270_mirror = 0x07

class IR_sensor:
    __dev_addr = 0xA4

    __rflag = 0x03
    __wflag = 0x06

    def __init__(self, 
            dev="/dev/serial0",
            baud_rate=9600):
        self.__com = Serial(dev, baud_rate)

    def __calculate_checksum(self, frame):
        s = sum(frame)
        return s & 0xFF

    def __validate_checksum(self, data):
        return self.__calculate_checksum(data[0:-1]) == data[-1]
    
    def open(self):
        return self.__com.open()

    def isOpen(self):
        return self.__com.isOpen()

    def close(self):
        return self.__com.close()

    def read_reg(self, reg_start, num_regs=1):
        cmd = [self.__dev_addr, self.__rflag, reg_start, num_regs, 0]
        cmd[-1] = self.__calculate_checksum(cmd)
        
        self.__com.write(bytes(cmd))
        log.debug(f"{__name__} cmd: {cmd}")

        data = self.__com.read(4 + num_regs + 1)
        log.debug(f"{__name__} data: {list(data)}")

        if self.__validate_checksum(data):
            return data[4:-1]
        return None

    def write_reg(self, reg, data):
        cmd = [self.__dev_addr, self.__wflag, reg, data, 0]
        cmd[-1] = self.__calculate_checksum(cmd)
        
        self.__com.write(bytes(cmd))
        log.debug(f"{__name__} cmd: {cmd}")

        data = self.__com.read(4 + 1 + 1)
        log.debug(f"{__name__} echo: {list(data)}")

        return self.__validate_checksum(data)

    def reset_config(self):
        reset_config_flag = 0xAA
        self.write_reg(IR_sensor_Reg.config, reset_config_flag)
    
    def save_config(self):
        save_config_flag = 0x55
        self.write_reg(IR_sensor_Reg.config, save_config_flag)

    def 

    def baud_rate(self, baud_rate : IR_sensor_Baud_rate):
        if IR_sensor_Baud_rate.baud_2400 == baud_rate:
            self.baud_rate = 2400
        elif IR_sensor_Baud_rate.baud_4800 == baud_rate:
            self.baud_rate = 4800
        elif IR_sensor_Baud_rate.baud_9600 == baud_rate:
            self.baud_rate = 9600
        elif IR_sensor_Baud_rate.baud_19200 == baud_rate:
            self.baud_rate = 19200
        elif IR_sensor_Baud_rate.baud_38400 == baud_rate:
            self.baud_rate = 38400
        elif IR_sensor_Baud_rate.baud_57600 == baud_rate:
            self.baud_rate = 57600
        elif IR_sensor_Baud_rate.baud_115200 == baud_rate:
            self.baud_rate = 115200
        elif IR_sensor_Baud_rate.baud_230400 == baud_rate:
            self.baud_rate = 230400
        else:
            self.baud_rate = 115200
        return self.baud_rate

