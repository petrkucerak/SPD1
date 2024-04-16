import board
import busio
import digitalio
from datetime import datetime

import adafruit_rfm9x


# Define radio parameters.
RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.


# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
CS = digitalio.DigitalInOut(board.D25)
RESET = digitalio.DigitalInOut(board.D17)

# Initialize SPI bus.
spi = busio.SPI(board.D11, MOSI=board.D10, MISO=board.D9)

# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, # must specify parameters
                             preamble_length=8, # The length in bytes of the packet preamble
                             high_power=True, # Boolean to indicate a high power board
                             baudrate=5000000, # Baud rate of the SPI connection
                             agc=False, # Boolean to Enable/Disable Automatic Gain Control - Default=False (AGC off)
                             crc=True # Boolean to Enable/Disable Cyclic Redundancy Check - Default=True (CRC Enabled)
                             )

rfm9x.tx_power = 23
rfm9x.signal_bandwidth = 500000
rfm9x.spreading_factor = 7
rfm9x.coding_rate = 5 # 4/5

# Send a packet.  Note you can only send a packet up to 252 bytes in length.
# This is a limitation of the radio packet size, so if you need to send larger
# amounts of data you will need to break it into smaller send calls.  Each send
# call will wait for the previous one to finish before continuing.
rfm9x.send(bytes("Hello world!\r\n", "utf-8"))
print("Sent Hello World message!")

# Wait to receive packets.  Note that this library can't receive data at a fast
# rate, in fact it can only receive and process one 252 byte packet at a time.
# This means you should only use this for low bandwidth scenarios, like sending
# and receiving a single message at a time.
print("Waiting for packets...")

while True:
    packet = rfm9x.receive(
        keep_listening=True,
        timeout=20.0,
        with_header=True
        )
    # Optionally change the receive timeout from its default of 0.5 seconds:
    # packet = rfm9x.receive(timeout=5.0)
    # If no packet was received during the timeout then None is returned.
    if packet is None:
        # Packet has not been received
        # LED.value = False
        now = datetime.now()
        print(now.strftime("%H:%M:%S"), "|", "Received nothing! Listening again...")
    else:
        # Received a packet!
        # LED.value = True
        # Print out the raw bytes of the packet:
        print("Received (raw bytes): {0}".format(packet))
        # And decode to ASCII text and print it too.  Note that you always
        # receive raw bytes and need to convert to a text format like ASCII
        # if you intend to do string processing on your data.  Make sure the
        # sending side is sending ASCII data before you try to decode!
        packet_text = str(packet, "ascii")
        print("Received (ASCII): {0}".format(packet_text))
        # Also read the RSSI (signal strength) of the last received message and
        # print it.
        rssi = rfm9x.last_rssi
        print("Received signal strength: {0} dB".format(rssi))