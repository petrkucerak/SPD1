from custom_lora import LoRa, ModemConfig
import board

# This is our callback function that runs when a message is received
def on_recv(payload):
    print("From:", payload.header_from)
    print("Received:", payload.message)
    print("RSSI: {}; SNR: {}".format(payload.rssi, payload.snr))

# Use chip select 0. GPIO pin 17 will be used for interrupts
# The address of this device will be set to 2
lora = LoRa(
    channel=0, # channel SPI channel to use (either 0 or 1, if your LoRa radio is connected to CE0 or CE1, respectively)
    interrupt=25, # GPIO pin (BCM-style numbering) to use for the interrupt
    this_address=2, # The address number (0-254) your device will use when sending and receiving packets.
    freq=868, # Frequency used by your LoRa radio. Defaults to 915Mhz
    tx_power=14, # Transmission power level from 5 to 23. Keep this as low as possible. Defaults to 14.
    modem_config=ModemConfig.Bw500Cr45Sf128, # Modem configuration. See RadioHead docs. Default to Bw125Cr45Sf128.
    receive_all=True, # Receive messages regardless of the destination address
    acks=False, # If True, send an acknowledgment packet when a message is received and wait for an acknowledgment when transmitting a message. This is equivalent to using RadioHead's RHReliableDatagram
    crypto=None # An instance of PyCryptodome Cipher.AES (see above example)
    )
lora.on_recv = on_recv

lora.set_mode_rx()

# Send a message to a recipient device with address 10
# Retry sending the message twice if we don't get an  acknowledgment from the recipient
message = "Hello there!"
status = lora.send_to_wait(message, 10, retries=2)
if status is True:
    print("Message sent!")
else:
    print("No acknowledgment from recipient")
    
# And remember to call this as your program exits...
lora.close()
