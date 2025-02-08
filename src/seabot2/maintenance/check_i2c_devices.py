import smbus2
import time
class SeabotDevice:
    def __init__(self):
        self.i2c_devices = {}

    def add_i2c_device(self, name, bus, addr):
        # Add a device to the dictionary (name: (bus, addr, alive))
        self.i2c_devices[name] = [bus, addr, False]

    def is_device_alive(self, name):
        bus, addr, _ = self.i2c_devices[name]
        alive = False
        try:
            with smbus2.SMBus(bus) as bus:
                bus.write_quick(addr)
                alive = True
        except OSError:
            pass
        self.i2c_devices[name][2] = alive

    def scan_devices(self):
        for name in self.i2c_devices:
            self.is_device_alive(name)

    def print_devices(self):
        # Sort devices by bus and address
        devices = sorted(self.i2c_devices.items(), key=lambda x: (x[1][0], x[1][1]))

        # Print devices with status (add a green checkmark if alive, red cross otherwise)
        for name, (bus, addr, alive) in devices:
            status = "\033[92m✅\033[0m" if alive else "\033[91m❌\033[0m"
            print(f"{status} [i2c-{bus}, 0x{addr:02X}] \t {name}")

if __name__ == "__main__":
    i2c = SeabotDevice()
    i2c.add_i2c_device("Internal Pressure (BME280)", 0, 0x77)
    i2c.add_i2c_device("External Pressure (MS5803)", 0, 0x77)
    i2c.add_i2c_device("PIC ecran", 0, 0x3C)
    i2c.add_i2c_device("PIC light", 0, 0x28)
    i2c.add_i2c_device("Dspic acoustique", 0, 0x1A)
    i2c.add_i2c_device("Audio codec (TLV320ADC6120)", 0, 0x4E)

    i2c.add_i2c_device("External Temperature (TSYS01)", 1, 0x76)
    i2c.add_i2c_device("PIC alim", 1, 0x39)
    i2c.add_i2c_device("PIC propulseurs", 1, 0x20)
    i2c.add_i2c_device("Dspic piston", 1, 0x1E)

    i2c.scan_devices()
    i2c.print_devices()