import board
import busio
import digitalio
from datetime import datetime
import os
import json
import time
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

print("Waiting for packets...")

# Save log as a txt
if not os.path.exists("../logs"):
    os.makedirs("../logs")

def run():
    while True:
        # Attempt to receive a packet with a timeout of 20 seconds
        packet = rfm9x.receive(keep_listening=True, timeout=20.0, with_header=True)
        now = datetime.now() # Get the current time
        if packet is None:
            # No packet received
            print(now.strftime("%H:%M:%S"), "|",
                "Received nothing! Listening again...")
        else:
            # Packet received
            # 2b - message_id (binary) | 2b - message_size (binary) | message (string) | 1b - checksum (binary modulo 255) | 1b - bitwise parity
            print()
            print("Packet:", packet)
            message_id = int.from_bytes(packet[:2], byteorder='big', signed=False)
            message_size = int.from_bytes(packet[2:4], byteorder='big', signed=False)
            message_end = message_size - 2
            message = str(packet[4:message_end], "utf-8")
            checksum = int.from_bytes(packet[message_end:message_end+1], byteorder='big', signed=False)
            bwp = int.from_bytes(packet[message_end+1:message_end+2], byteorder='big', signed=False)


            print("Message ID:", message_id)
            print("Message size:", message_size)
            print("Message:", message)
            print("Checksum:", checksum)
            print("Bwp:", bwp)

            # Calculate checksum
            for i in range(message_end + 1):
                checksum_local = int(packet[i])
            checksum_local = checksum_local % 256
            print("Checksum local:", checksum_local)

            if(checksum_local != checksum):
                print("ERROR: wrong checksum")
                continue

            # Calculate bitwise parity
            bwp_local = 0
            for i in range(message_size - 1):
                bwp_local = bwp_local ^ packet[i]
            print("Bitwise parity local:", bwp_local)

            if(bwp_local != bwp):
                print("ERROR: wrong bwp")
                continue


            # Send ACK
            ack = bytearray(7)
            ack[:2] = packet[:2]
            ack[2] = 0x41 # 'A'
            ack[3] = 0x43 # 'C'
            ack[4] = 0x4B # 'K'
            ack[5] = checksum_local
            ack[6] = bwp_local

            rfm9x.send(bytes(ack))
        

            # Save log to file
            pathname = f'../logs/{now.strftime("%Y-%m-%d")}.log'
            data = message.replace("\n\r\x00", "").split(";")
            json_data = {}
            for i in data:
                key, value = i.split("=")
                json_data[key] = value
            json_data["time"] = now.strftime("%Y-%m-%d %H:%M:%S")
            print(json_data)
            with open(pathname, "a+") as f:
                f.write(json.dumps(json_data) + ",\n")
    pass

# Main loop to run the script, restarting if an error occurs
while True:
    try:
        run()
        break
    except Exception as e:
        print(f"Error occurred: {e}. Restarting the script...")
        time.sleep(5)
