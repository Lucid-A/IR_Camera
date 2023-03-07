from enum import IntEnum
from serial import Serial
import logging as logger
from math import log10


logger.basicConfig(
        level=logger.DEBUG,
#       filename='IR_sensor.log',
#       filemode='w'
        )


class IR_sensor_Reg(IntEnum):
    addr_ID              = 0x00
    baud_rate            = 0x01
    update_hz            = 0x02
    output_mode          = 0x03
    img_output_fmt       = 0x04
    config               = 0x05
    temp_compsente       = 0x06
    emissivity           = 0x07
    threshold            = 0x08
    ta_h                 = 0x09
    ta_l                 = 0x0A
    to_h8_start          = 0x0B
    to_h8_end            = 0x4A
    to_l8_start          = 0x4B
    to_l8_end            = 0x8A
    to_over_thresh_start = 0x8B
    to_over_thresh_end   = 0x92
    max_temp_h8          = 0x93
    max_temp_l8          = 0x94
    min_temp_h8          = 0x95
    min_temp_l8          = 0x96
    firmware             = 0x97


class IR_sensor_Baud_rate(IntEnum):
    baud_2400   = 0x00
    baud_4800   = 0x01
    baud_9600   = 0x02
    baud_19200  = 0x03
    baud_38400  = 0x04
    baud_57600  = 0x05
    baud_115200 = 0x06
    baud_230400 = 0x07


IR_baud = {
    IR_sensor_Baud_rate.baud_2400   : 2400,
    IR_sensor_Baud_rate.baud_4800   : 4800,
    IR_sensor_Baud_rate.baud_9600   : 9600,
    IR_sensor_Baud_rate.baud_19200  : 19200,
    IR_sensor_Baud_rate.baud_38400  : 38400,
    IR_sensor_Baud_rate.baud_57600  : 57600,
    IR_sensor_Baud_rate.baud_115200 : 115200,
    IR_sensor_Baud_rate.baud_230400 : 230400
}


class IR_sensor_Output_mode(IntEnum):
    continuous   = 0x00
    query        = 0x01
    auto_char    = 0x02


class IR_sensor_Img_transform(IntEnum):
    origin = 0x00
    rotate_clockwise_90     = 0x01
    rotate_clockwise_180    = 0x02
    rotate_clockwise_270    = 0x03
    mirror = 0x04
    rotate_clockwise_90_mirror  = 0x05
    rotate_clockwise_180_mirror = 0x06
    rotate_clockwise_270_mirror = 0x07


class IR_sensor:
    __rflag = 0x03
    __wflag = 0x06


    def __init__(self, 
            dev="/dev/serial0",
            baud_rate=9600):
        self.__dev_addr = 0xA4
        self.baud_rate = 9600
        self.update_hz = 10
        self.output_mode = IR_sensor_Output_mode.query
        self.img_output_fmt = IR_sensor_Img_transform.origin
        self.temp_offset_compsente = 0
        self.emissivity = 1.00
        self.__com = Serial(dev, baud_rate)

        if self.__com.isOpen():
            self.__com.close()

        self.__com.open()
        self.write_reg(IR_sensor_Reg.output_mode, self.output_mode)
        self.write_reg(IR_sensor_Reg.img_output_fmt, self.img_output_fmt)
        self.close()


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
        logger.debug(f"{__name__} cmd: {cmd}")

        data = self.__com.read(4 + num_regs + 1)
        logger.debug(f"{__name__} data: {list(data)}")

        if self.__validate_checksum(data):
            return data[4:-1]
        return None


    def write_reg(self, reg, data):
        cmd = [self.__dev_addr, self.__wflag, reg, data, 0]
        cmd[-1] = self.__calculate_checksum(cmd)
        
        self.__com.write(bytes(cmd))
        logger.debug(f"{__name__} cmd: {cmd}")

        data = self.__com.read(4 + 1 + 1)
        logger.debug(f"{__name__} echo: {list(data)}")

        return self.__validate_checksum(data)


    def reset_config(self):
        reset_config_flag = 0xAA
        self.write_reg(IR_sensor_Reg.config, reset_config_flag)
    

    def save_config(self):
        save_config_flag = 0x55
        self.write_reg(IR_sensor_Reg.config, save_config_flag)


    def set_baud_rate(self, baud_rate : IR_sensor_Baud_rate, save_config=False):
        if baud_rate not in IR_baud:
            baud_rate = IR_sensor_Baud_rate.baud_115200
        
        self.baud_rate = IR_baud[baud_rate]
        success = self.write_reg(IR_sensor_Reg.baud_rate, baud_rate)

        if success:
            print(f"Set Baud Rate: {baud_rate} [/] {self.baud_rate} baud/s")

        if save_config:
            self.save_config()
            
        return self.baud_rate


    def get_baud_rate(self):
        data = self.read_reg(IR_sensor_Reg.baud_rate)
        if data is not None:
            data = data[0]
            if data in IR_baud:
                self.baud_rate = IR_baud[data]
                return self.baud_rate

        self.baud_rate = 0
        return 0


    def set_update_hz(self, update_hz=10, save_config=False):
        self.update_hz = int(log10(update_hz))
        success = self.write_reg(IR_sensor_Reg.update_hz, self.update_hz)
        self.update_hz = 10**self.update_hz
        

        if success:
            print(f"Set Update Frequency: {self.update_hz} Hz")

        if save_config:
            self.save_config()
            
        return self.update_hz

    def get_update_hz(self):
        data = self.read_reg(IR_sensor_Reg.update_hz)
        if data is not None:
            data = data[0]
            self.update_hz = 10 ** data
            return self.update_hz

        self.update_hz = 0
        return self.update_hz

    def get_temp_map(self, resolution_high=False):
        img = None

        img_h = self.read_reg(IR_sensor_Reg.to_h8_start, 64)
        if img_h is not None:
            img = [(t + 127) for t in img_h]
            #img = img_h

        if resolution_high:
            img_l = self.read_reg(IR_sensor_Reg.to_l8_start, 64)
            if img_l is not None:
                img = [(t[0] + t[1] * 0.25) for t in zip(img, img_l)]
        
        return img
        
