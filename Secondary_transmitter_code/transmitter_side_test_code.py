from machine import Pin, SPI
from utime import sleep
from nrf24 import NRF24L01

TX_ADDR = b"\xe1\xf0\xf0\xf0\xf0"
RX_ADDR = b"\xd2\xf0\xf0\xf0\xf0"

# SPI pins
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
csn = Pin(5, Pin.OUT)
ce = Pin(6, Pin.OUT)

# Initialize NRF24
nrf = NRF24L01(spi, csn, ce, channel=76, payload_size=32)

# TX pipe
nrf.open_tx_pipe(TX_ADDR)
# RX pipe is needed by library but won't be used here
nrf.open_rx_pipe(0, RX_ADDR)

# Disable ACK (no receiver required)

nrf.stop_listening()
sleep(0.05)

print("Transmitter ready...")

count = 0
while True:
    count += 1
    payload = f"Message {count}".encode()[:32]
    try:
        nrf.send(payload)
        print("Sent:", payload)
    except OSError as e:
        print("Send failed:", e)
    sleep(1)  # send 1 message per second

