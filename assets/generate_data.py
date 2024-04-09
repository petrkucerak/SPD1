import sys
import json
import numpy as np
import random


def make_tmp_hop(temperature: float, ub: float, lb: float):
    if random.randint(0, 1):
        temperature += random.uniform(0.5, 2.5)
    else:
        temperature -= random.uniform(0.5, 2.5)
    if temperature < lb:
        temperature = lb
    if temperature > ub:
        temperature = ub
    return temperature


if len(sys.argv) < 3:
    print("Please specific parameters! (For example generate_data.py <records count> <output file path>)")
    exit(1)

records = int(sys.argv[1])
path = sys.argv[2]

w_temperature = 4.71  # start water temperature
a_temperature = 8.25  # start air temperature
moist_air = 57.4  # start air moist
start_time = 1388139018

data = []
for i in range(records):
    w_temperature = make_tmp_hop(w_temperature, lb=4, ub=20)
    a_temperature = make_tmp_hop(a_temperature, ub=30, lb=-8)
    moist_air = make_tmp_hop(a_temperature, ub=98, lb=0)
    start_time += 3600
    data.append({'t_water': w_temperature, "t_air": a_temperature, "moist_air": moist_air,
                "light": random.randint(1000, 6000), "gps_lat": 50.088032, "gps_lan": 14.452941, "gps_time": start_time})

with open(path, "w+") as f:
    json.dump(data, f)
