# mpu.py - MicroPython driver for MPU6050 (Pico-safe)
# Original: GY-521 module
# Ported & simplified to avoid automatic SoftI2C creation

from math import sqrt, atan2
from machine import Pin
from time import sleep_ms

# Constants
_GRAVITY_MS2 = 9.80665

# Accelerometer scale modifiers
_ACC_SCLR_2G = 16384.0
_ACC_SCLR_4G = 8192.0
_ACC_SCLR_8G = 4096.0
_ACC_SCLR_16G = 2048.0

# Gyroscope scale modifiers
_GYR_SCLR_250DEG = 131.0
_GYR_SCLR_500DEG = 65.5
_GYR_SCLR_1000DEG = 32.8
_GYR_SCLR_2000DEG = 16.4

# Register addresses
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT0 = 0x3B
_TEMP_OUT0 = 0x41
_GYRO_XOUT0 = 0x43
_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B

# Default I2C address
_MPU6050_ADDRESS = 0x68

# Helper
def signedIntFromBytes(x, endian="big"):
    y = int.from_bytes(x, endian)
    return y - 65536 if y >= 0x8000 else y

class MPU6050:
    def __init__(self, i2c, addr=_MPU6050_ADDRESS):
        """Initialize with an existing I2C bus"""
        self.i2c = i2c
        self.addr = addr
        self._failCount = 0
        self._terminatingFailCount = 0

        # Wake up MPU6050
        self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, bytes([0x00]))
        sleep_ms(5)

        self._accel_range = self.get_accel_range(True)
        self._gyro_range = self.get_gyro_range(True)

    def _readData(self, reg):
        """Read 6 bytes of sensor data"""
        try:
            data = self.i2c.readfrom_mem(self.addr, reg, 6)
        except OSError:
            return {"x": float("NaN"), "y": float("NaN"), "z": float("NaN")}
        x = signedIntFromBytes(data[0:2])
        y = signedIntFromBytes(data[2:4])
        z = signedIntFromBytes(data[4:6])
        return {"x": x, "y": y, "z": z}

    # Accelerometer
    def read_accel_data(self, g=False):
        d = self._readData(_ACCEL_XOUT0)
        # Choose scaler based on set range
        scaler = {
            0x00: _ACC_SCLR_2G,
            0x08: _ACC_SCLR_4G,
            0x10: _ACC_SCLR_8G,
            0x18: _ACC_SCLR_16G
        }.get(self._accel_range, _ACC_SCLR_2G)

        x, y, z = d["x"]/scaler, d["y"]/scaler, d["z"]/scaler
        if g:
            return {"x": x, "y": y, "z": z}
        else:
            return {"x": x*_GRAVITY_MS2, "y": y*_GRAVITY_MS2, "z": z*_GRAVITY_MS2}

    def read_accel_abs(self, g=False):
        d = self.read_accel_data(g)
        return sqrt(d["x"]**2 + d["y"]**2 + d["z"]**2)

    def set_accel_range(self, accel_range):
        self.i2c.writeto_mem(self.addr, _ACCEL_CONFIG, bytes([accel_range]))
        self._accel_range = accel_range

    def get_accel_range(self, raw=False):
        raw_data = self.i2c.readfrom_mem(self.addr, _ACCEL_CONFIG, 1)
        if raw:
            return raw_data[0]
        return {0x00:2, 0x08:4, 0x10:8, 0x18:16}.get(raw_data[0], -1)

    # Gyroscope
    def read_gyro_data(self):
        d = self._readData(_GYRO_XOUT0)
        scaler = {
            0x00: _GYR_SCLR_250DEG,
            0x08: _GYR_SCLR_500DEG,
            0x10: _GYR_SCLR_1000DEG,
            0x18: _GYR_SCLR_2000DEG
        }.get(self._gyro_range, _GYR_SCLR_250DEG)
        return {"x": d["x"]/scaler, "y": d["y"]/scaler, "z": d["z"]/scaler}

    def set_gyro_range(self, gyro_range):
        self.i2c.writeto_mem(self.addr, _GYRO_CONFIG, bytes([gyro_range]))
        self._gyro_range = gyro_range

    def get_gyro_range(self, raw=False):
        raw_data = self.i2c.readfrom_mem(self.addr, _GYRO_CONFIG, 1)
        if raw:
            return raw_data[0]
        return {0x00:250, 0x08:500, 0x10:1000, 0x18:2000}.get(raw_data[0], -1)

    # Temperature
    def read_temperature(self):
        try:
            raw = self.i2c.readfrom_mem(self.addr, _TEMP_OUT0, 2)
            raw_val = signedIntFromBytes(raw)
        except OSError:
            return float("NaN")
        return raw_val/340 + 36.53

    # Optional: simple angle approximation
    def read_angle(self):
        a = self.read_accel_data()
        return {"x": atan2(a["y"], a["z"]), "y": atan2(-a["x"], a["z"])}
