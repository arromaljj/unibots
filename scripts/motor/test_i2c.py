from smbus import SMBus
import time

# Try multiple buses
for bus_num in [0, 1, 2, 3, 8]:
    try:
        print(f"Testing bus {bus_num}...")
        bus = SMBus(bus_num)
        
        # Try BNO055 default address (0x28)
        try:
            value = bus.read_byte_data(0x28, 0x00)
            print(f"  Success at 0x28! Read value: {value}")
        except Exception as e:
            print(f"  Error at 0x28: {e}")
        
        # Try alternative address (0x29)
        try:
            value = bus.read_byte_data(0x29, 0x00)
            print(f"  Success at 0x29! Read value: {value}")
        except Exception as e:
            print(f"  Error at 0x29: {e}")
        
        bus.close()
    except Exception as e:
        print(f"Error opening bus {bus_num}: {e}")

print("Test complete")
