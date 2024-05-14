import board
import busio
import digitalio
from datetime import datetime
import os
import json

import adafruit_rfm9x

# Define radio parameters.
RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in Mhz. Must match your

# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
CS = digitalio.DigitalInOut(board.D25)
RESET = digitalio.DigitalInOut(board.D17)

# Initialize SPI bus.
spi = busio.SPI(board.D11, MOSI=board.D10, MISO=board.D9)

# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ,  # must specify parameters
                             preamble_length=8,  # The length in bytes of the packet preamble
                             high_power=True,  # Boolean to indicate a high power board
                             baudrate=5000000,  # Baud rate of the SPI connection
                             # Boolean to Enable/Disable Automatic Gain Control - Default=False (AGC off)
                             agc=False,
                             # Boolean to Enable/Disable Cyclic Redundancy Check - Default=True (CRC Enabled)
                             crc=True
                             )

rfm9x.tx_power = 23
rfm9x.signal_bandwidth = 500000
rfm9x.spreading_factor = 7
rfm9x.coding_rate = 5  # 4/5

# Send a packet.  Note you can only send a packet up to 252 bytes in length.
# This is a limitation of the radio packet size, so if you need to send larger
# amounts of data you will need to break it into smaller send calls.  Each send
# call will wait for the previous one to finish before continuing.
rfm9x.send(bytes("Hello cold world!\r\n", "utf-8"))
print("Sent Hello cold world message!")

# Wait to receive packets.  Note that this library can't receive data at a fast
# rate, in fact it can only receive and process one 252 byte packet at a time.
# This means you should only use this for low bandwidth scenarios, like sending
# and receiving a single message at a time.
print("Waiting for packets...")

# Save log as a txt
if not os.path.exists("../logs"):
    os.makedirs("../logs")

while True:
    packet = rfm9x.receive(keep_listening=True, timeout=20.0, with_header=True)
    # Optionally change the receive timeout from its default of 0.5 seconds:
    # packet = rfm9x.receive(timeout=5.0)
    # If no packet was received during the timeout then None is returned.
    now = datetime.now()
    if packet is None:
        # Packet has not been received
        print(now.strftime("%H:%M:%S"), "|",
              "Received nothing! Listening again...")
    else:
        # Received a packet!

        print(now.strftime("%H:%M:%S"), "|",
              "Received (raw bytes): {0}".format(packet))
        

        # Packet formate
        # | 2B - message_id (binary) | message_size (binary) | message (string) | checksum (binary modulo 255) | parita
        packet = 0x0102
        message_id = int.from_bytes(packet[:2])
        message_size = int.from_bytes(packet[2:4])
        message_end = message_size + 4
        message = int.from_bytes(packet[4:message_end])
        checksum = int.from_bytes(packet[message_end:message_end+1])
        bwp = int.from_bytes(packet[message_end+1:message_end+2])


        print(message_id, message_size, message_size, message_end, message, checksum, bwp)

        packet_text = str(packet, "ascii")
        
        print(now.strftime("%H:%M:%S"), "|",
              "Received (ASCII): {0}".format(packet_text))

        # save log to file
        # pathname = f'../logs/{now.strftime("%Y-%m-%d")}.log'
        # data = packet_text.replace("\n\r\x00", "").split(";")
        # json_data = {}
        # for i in data:
        #     key, value = i.split("=")
        #     json_data[key] = value
        # print(json_data)
        # with open(pathname, "a+") as f:
        #     f.write(json.dumps(json_data) + ",\n")
