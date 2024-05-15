# Gateway device

LoRa Gateway device runs on Raspberry PI.

## Connect to Raspberry PI

```sh
ssh pedro@192.168.0.178
ssh pedro@192.168.137.157
K&mo1234 # password
```

## Hardware

### Lora shield

- LoRa Gateway RPI/RFI
- pinout: https://electronilab.co/tienda/modulo-transceptor-rfm96w-hoperf-lora-ultra-long-range-433-mhz/
- datasheet: https://cdn-learn.adafruit.com/assets/assets/000/031/660/original/sx1272.pdf?1460518722


```
    H
   rf96
 17081714
1708122155
```



## Software

Uses https://github.com/adafruit/Adafruit_CircuitPython_RFM9x

```sh
mkdir project-name && cd project-name
python3 -m venv .venv
source .venv/bin/activate
pip3 install adafruit-circuitpython-rfm9x
```