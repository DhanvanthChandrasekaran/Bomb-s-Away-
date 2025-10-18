import utime
import ustruct as struct
from machine import Pin, SPI, I2C, PWM
import os
from nrf24 import NRF24L01
import sdcard
import mpu6050
from ms4525 import MS4525DO

#telemetry logging params
telem_interval = 200 #in ms
rx_window = 80 #in ms
servo_release = 600 #also in ms (no surprise)

TX_ADDR = b"\xe1\xf0\xf0\xf0\xf0"
RX_ADDR = b"\xd2\xf0\xf0\xf0\xf0" #this is called little-endian encoding, no idea what that means
RELEASE_COMMAND =  0x01 #RELEASE THE PAYLOAADDDDDD

#general spi interface params
SPI_id = 0
sck = 2
mosi = 3
miso = 4

# SDcard chip select and nrf24 chip select, chip enable pins
nrf_csn = 5
nrf_ce = 6
sdCard_cs = 1

#mpu6050 params
imu_scl = 17
imu_sda = 16
imu_addr = 0x68

servoPin = Pin(28, Pin.OUT)
masterSwitchPin = Pin(22, Pin.IN)

spi = SPI(SPI_id,  sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))

#sdcard init
card = sdcard.SDCard(spi, Pin(sdCard_cs, Pin.OUT, value=1))
vfs = os.VfsFat(card)
os.mount(vfs, "/card")
LOG_PATH = "/card/telemetry.csv"

#nrf init
csn = Pin(nrf_csn, mode=Pin.OUT, value=1)
ce = Pin(nrf_ce, mode=Pin.OUT, value=0)
nrf = NRF24L01(spi, csn, ce, channel=76, payload_size=32)
nrf.open_tx_pipe(TX_ADDR)
nrf.open_rx_pipe(1, RX_ADDR)

#mpu6050 init
i2c = I2C(0, scl=Pin(imu_scl), sda=Pin(imu_sda), freq=400000)
mpu = MPU6050(i2c)
mpu.set_accel_range(0x00)  # +-2g's
mpu.set_gyro_range(0x00)   # +-250 degrees/sec

#airspeed sensor init
airspeed_sensor = MS4525DO(i2c, addr=0x28, p_max=1.0, p_min=-1.0)

#servo and master switch pwm init
servo_pwm = PWM(Pin(servoPin))
servo_pwm.freq(50)
master_switch = Pin(masterSwitchPin, Pin.IN)

def servo_angle(angle):
    min_pulse = 0.025
    max_pulse = 0.125
    frac = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
    servo_pwm.duty_u16(int(frac * 65535))

def perform_release():
    servo_angle(90)
    utime.sleep_ms(servo_release)
    servo_angle(0)

def append_log(line):
    try:
        with open(LOG_PATH, "a") as f:
            f.write(line + "\n")
    except:
        pass

def handle_command(cmd_byte):
    if cmd_byte == RELEASE_COMMAND:
        if master_switch.value():
            print("Master switch ON — releasing servo")
            perform_release()
        else:
            print("Release command ignored (master switch OFF)")

#void setup()
servo_angle(0)
print("HELLLLOOOOOOOOOOOOOOOO, logging to ", LOG_PATH)
nrf.start_listening()

last_send = utime.ticks_ms()

while True:
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_send) >= telem_interval:
        accel = mpu.read_accel_data(g=True)
        gyro = mpu.read_gyro_data()
        temp = mpu.read_temperature()
        ts = utime.ticks_ms()

        ok = airspeed_sensor.read()
        pressure_pa = airspeed_sensor.pres_pa if ok else 0.0
        # Airspeed from differential pressure
        # rho = 1.225 (air density, kg/m³)
        airspeed = (2 * abs(pressure_pa) / 1.225) ** 0.5 if pressure_pa > 0 else 0.0

        csv = "{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{}".format(
            ts, accel["x"], accel["y"], accel["z"],
            gyro["x"], gyro["y"], gyro["z"], pressure_pa, airspeed,temp, master_switch.value()
        )
        append_log(csv)

        payload = csv.encode()[:32]
        nrf.stop_listening()
        try:
            nrf.send(payload)
        except OSError:
            pass
        nrf.start_listening()

        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < rx_window:
            if nrf.any():
                while nrf.any():
                    pkt = nrf.recv()
                    if len(pkt) >= 1:
                        handle_command(pkt[0])
            utime.sleep_ms(5)

        last_send = now
    else:
        utime.sleep_ms(5)
