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

# Open RX pipe
nrf.open_rx_pipe(1, RX_ADDR)
# TX pipe must exist for ACK handling
nrf.open_tx_pipe(TX_ADDR)

# Enable auto-ack (optional)

nrf.start_listening()
sleep(0.05)

print("Receiver ready...")

while True:
    if nrf.any():
        while nrf.any():
            msg = nrf.recv()
            msg = msg.rstrip(b"\x00")  # strip padding
            print("Received:", msg)
    sleep(0.1)
