import utime
from machine import Pin, SPI
from nrf24 import NRF24L01
utime.sleep(0.1) # Wait for USB to become ready

#addresses
RX_ADDR = b"\xe1\xf0\xf0\xf0\xf0"
TX_ADDR = b"\xd2\xf0\xf0\xf0\xf0"
CMD_RELEASE = 0x01

#spi initialisation for nrf24
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
csn = Pin(5, mode=Pin.OUT)
ce = Pin(6, mode=Pin.OUT)

#nrf24 init stuff
nrf = NRF24L01(spi, csn, ce, channel=76, payload_size=32)
nrf.open_tx_pipe(TX_ADDR)
nrf.open_rx_pipe(1, RX_ADDR)
nrf.start_listening()

#trigger push button logic init
triggerOldstate = 1
count = 0
triggerPin = Pin(28, Pin.IN)

while True:
    if nrf.any():
        while nrf.any():
            pkt = nrf.recv()
            msg = pkt.rstrip(b"\x00").decode(errors="ignore")
            count += 1
            print(f"[{count}] {msg}")
    
    triggerNewstate = triggerPin.value()
    if triggerNewstate==0 and triggerOldstate==1:
        print("Button pressed, ending release command")
        nrf.stop_listening()
        nrf.send(bytes([CMD_RELEASE]) + b"\x00" * 31)
        nrf.start_listening()
        utime.sleep_ms(100)
    triggerOldstate = triggerNewstate
    utime.sleep_ms(10)
