import time
from rover.pac1720 import PAC1720, _config_val, _reg

if __name__ == '__main__':
    sensor2 = PAC1720(6, 0x4f)
    sensor2.begin()

    while True:
        vb1, vb2 = sensor2.get_bus_voltage(1), sensor2.get_bus_voltage(2)
        v1, i1 = sensor2.get_sense_voltage(1), sensor2.get_current(1)
        v2, i2 = sensor2.get_sense_voltage(2), sensor2.get_current(2)
        
        print(f'Channel 1: {vb1:.2f} Vsource {v1:.2f} Vsense {i1:.5f} A')
        print(f'Channel 2: {vb2:.2f} Vsource {v2:.2f} Vsense {i2:.5f} A')
        print()
        time.sleep(0.5)