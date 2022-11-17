from smbus import SMBus
from enum import IntEnum
import ctypes as ct

class _reg(IntEnum):
    Configuration = 0x00
    Conversion_Rate = 0x01
    One_Shot = 0x02
    Channel_Mask_Register = 0x03
    High_Limit_Status = 0x04
    Low_Limit_Status = 0x05
    VSOURCE_Sampling = 0x0A
    CH1_VSENSE_Sampling = 0x0B
    CH2_VSENSE_Sampling = 0x0C
    CH1_Sense_Voltage_High = 0x0D
    CH1_Sense_Voltage_Low = 0x0E
    CH2_Sense_Voltage_High = 0x0F
    CH2_Sense_Voltage_Low = 0x10
    CH1_VSOURCE_Voltage_High = 0x11
    CH1_VSOURCE_Voltage_Low = 0x12
    CH2_VSOURCE_Voltage_High = 0x13
    CH2_VSOURCE_Voltage_Low = 0x14
    CH1_Power_Ratio_High = 0x15
    CH1_Power_Ratio_Low = 0x16
    CH2_Power_Ratio_High = 0x17
    CH2_Power_Ratio_Low = 0x18
    CH1_Sense_Voltage_High_Limit = 0x19
    CH2_Sense_Voltage_High_Limit = 0x1A
    CH1_Sense_Voltage_Low_Limit = 0x1B
    CH2_Sense_Voltage_Low_Limit = 0x1C
    CH1_VSOURCE_Voltage_High_Limit = 0x1D
    CH2_VSOURCE_Voltage_High_Limit = 0x1E
    CH1_VSOURCE_Voltage_Low_Limit = 0x1F
    CH2_VSOURCE_Voltage_Low_Limit = 0x20
    Product_ID = 0xFD
    Manufacturer_ID = 0xFE
    Revision = 0xFF


class _config_val(IntEnum):
    """Top byte is reg address, lower byte is shift and mask
    """
    C1RS = 0x0A32
    C1RA = 0x0A30
    C2RS = 0x0A36
    C2RA = 0x0A34
    C1CSS = 0x0B74
    C1SA = 0x0B32
    C1SR = 0x0B30
    C2CSS = 0x0C74
    C2SA = 0x0C32
    C2SR = 0x0C30


class PAC1720Exception(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class PAC1720:
    _address: int
    _r_sense = 0.02  # sense resistor is 0.02 ohms

    def __init__(self, port, address=0x4C) -> None:
        self._bus = SMBus(port)
        self._address = address
    
    def _read_byte_data(self, reg: _reg) -> int:
        return self._bus.read_byte_data(self._address, reg)

    def _write_byte_data(self, reg: _reg, data: int) -> None:
        self._bus.write_byte_data(self._address, reg, data)
    
    def _read_word_data(self, reg: _reg) -> int:
        return self._bus.read_word_data(self._address, reg)
    
    def _write_word_data(self, reg: _reg, data: int) -> None:
        self._bus.write_word_data(self._address, reg, data)
    
    def _read_i2c_block_data(self, reg: _reg, size: int) -> 'list[int]':
        return self._bus.read_byte_data(self._address, reg, size)

    def _write_i2c_block_data(self, reg: _reg, block: 'list[int]') -> None:
        self._bus.write_i2c_block_data(self._address, reg, block)
    
    def _get_config(self, config_val: _config_val) -> int:
        reg = _reg(config_val >> 8)
        offset = config_val & 0x0F
        mask = (config_val >> 4) & 0x0F

        data = self._read_byte_data(reg)

        return (data >> offset) & mask

    def _set_config(self, config_val: _config_val, new_val: int) -> bool:
        reg = _reg(config_val >> 8)
        offset = config_val & 0x0F
        mask = (config_val >> 4) & 0x0F

        prev_val = self._read_byte_data(reg)
        prev_val = prev_val & ~(mask << offset)

        write_val = (new_val << offset) | prev_val
        self._write_byte_data(reg, write_val)

        return write_val == self._read_byte_data(reg)
    
    def begin(self) -> bool:
        error = False  # Default to no error 
        # Setup device for default operation
        # No averaging on CH1 bus measurment 
        error |= self._set_config(_config_val.C1RA, 0x00)
        # No averaging on CH2 bus measurment 
        error |= self._set_config(_config_val.C2RA, 0x00)
        # 11 bit sample on CH1 bus
        error |= self._set_config(_config_val.C1RS, 0b11)
        # 11 bit sample on CH2 bus
        error |= self._set_config(_config_val.C2RS, 0b11)
        # 11 bit sense, min sample time, on CH1 sense
        error |= self._set_config(_config_val.C1CSS, 0b101)
        # 11 bit sense, min sample time, on CH2 sense
        error |= self._set_config(_config_val.C2CSS, 0b101)
        # Set for 80mV FSR on CH1
        error |= self._set_config(_config_val.C1SR, 0b11)
        # Set for 80mV FSR on CH2
        error |= self._set_config(_config_val.C2SR, 0b11)
        # No averaging on CH1 sense measurment 
        error |= self._set_config(_config_val.C1SA, 0b00)
        # No averaging on CH2 sense measurment 
        error |= self._set_config(_config_val.C2SA, 0b00)

        return error
    
    def end(self) -> bool:
        # TODO: Write this function
        return True
        
    def get_bus_voltage(self, channel: int) -> float:
        den = 2048  # default
        den_bits = 3  # default

        if channel not in {1, 2}:
            raise PAC1720Exception(f"Cannot get voltage of channel {channel}")
        elif channel is 1:
            den_bits = self._get_config(_config_val.C1RS)
        else:
            den_bits = self._get_config(_config_val.C2RS)
        
        den = pow(2, den_bits + 8)
        FSV = 40.0 - 40.0 / den

        data_high: int
        data_low: int
        if channel is 1:
            data_high = self._read_byte_data(_reg.CH1_VSOURCE_Voltage_High)
            data_low = self._read_byte_data(_reg.CH1_VSOURCE_Voltage_Low)
        else:
            data_high = self._read_byte_data(_reg.CH2_VSOURCE_Voltage_High)
            data_low = self._read_byte_data(_reg.CH2_VSOURCE_Voltage_Low)
        
        vsource = ((data_high << 3) | data_low >> 5)
        return FSV * (vsource / (den - 1))
    
    def get_sense_voltage(self, channel: int) -> float:
        FSR = 80
        den = 2047

        if channel not in {1, 2}:
            raise PAC1720Exception(f"Cannot get voltage of channel {channel}")
        elif channel is 1:
            FSR = (2 ** self._get_config(_config_val.C1SR)) * 10
            den = 6 + self._get_config(_config_val.C1CSS)
        else:
            FSR = (2 ** self._get_config(_config_val.C2SR)) * 10
            den = 6 + self._get_config(_config_val.C2CSS)
        
        den = min(den, 11)
        den = (2 << den) - 1
        FSC = FSR

        data_high: int
        data_low: int
        if channel is 1:
            data_high = self._read_byte_data(_reg.CH1_Sense_Voltage_High)
            data_low = self._read_byte_data(_reg.CH1_Sense_Voltage_Low)
        else:
            data_high = self._read_byte_data(_reg.CH2_Sense_Voltage_High)
            data_low = self._read_byte_data(_reg.CH2_Sense_Voltage_Low)
        
        vsense = ct.c_short((data_high << 4) | (data_low >> 4))

        if vsense.value & 0x800:
            vsense.value |= 0xF000
        
        return FSC * (vsense.value / den)
    
    def get_current(self, channel: int) -> float:
        FSR = 80
        den = 2047

        if channel not in {1, 2}:
            raise PAC1720Exception(f"Cannot get voltage of channel {channel}")
        elif channel is 1:
            FSR = (2 ** self._get_config(_config_val.C1SR)) * 10
            den = 6 + self._get_config(_config_val.C1CSS)
        else:
            FSR = (2 ** self._get_config(_config_val.C2SR)) * 10
            den = 6 + self._get_config(_config_val.C2CSS)
        
        den = min(den, 11)
        den = (2 << (den - 1)) - 1
        FSC = (FSR / 1000) / self._r_sense

        data_high: int
        data_low: int
        if channel is 1:
            data_high = self._read_byte_data(_reg.CH1_Sense_Voltage_High)
            data_low = self._read_byte_data(_reg.CH1_Sense_Voltage_Low)
        else:
            data_high = self._read_byte_data(_reg.CH2_Sense_Voltage_High)
            data_low = self._read_byte_data(_reg.CH2_Sense_Voltage_Low)
        
        vsense = ct.c_short((data_high << 4) | (data_low >> 4))

        if vsense.value & 0x800:
            vsense.value |= 0xF000
        
        # breakpoint()

        return FSC * (vsense.value / den)
    
    def get_bus_power(self, channel: int) -> float:
        FSR = 80

        if channel not in {1, 2}:
            raise PAC1720Exception(f"Cannot get voltage of channel {channel}")
        elif channel is 1:
            FSR = (2 ** self._get_config(_config_val.C1SR)) * 10
        else:
            FSR = (2 ** self._get_config(_config_val.C2SR)) * 10
        