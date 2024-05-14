import sys
import json
import numpy as np
import random
import datetime


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
start_time = datetime.datetime.now()

w_temperature_2 = 5.71  # start water temperature
a_temperature_2 = 9.25  # start air temperature
moist_air_2 = 63.4  # start air moist
start_time_2 = datetime.datetime.now()

data = []
for i in range(records):
    w_temperature = make_tmp_hop(w_temperature, lb=4, ub=20)
    a_temperature = make_tmp_hop(a_temperature, ub=30, lb=-8)
    moist_air = make_tmp_hop(a_temperature, ub=98, lb=0)
    start_time += datetime.timedelta(seconds=11)
    data.append({
        'Tw': w_temperature,
        "Ta": a_temperature,
        "Ma": moist_air,
        "Lo": random.randint(1000, 6000),
        "GN": 50.0517678,
        "GE": 14.4965397,
        "time": str(start_time)
    })

    w_temperature_2 = make_tmp_hop(w_temperature_2, lb=4, ub=20)
    a_temperature_2 = make_tmp_hop(a_temperature_2, ub=30, lb=-8)
    moist_air_2 = make_tmp_hop(a_temperature_2, ub=98, lb=0)
    start_time_2 += datetime.timedelta(seconds=11)

    data.append({
        'Tw': w_temperature_2,
        "Ta": a_temperature_2,
        "Ma": moist_air_2,
        "Lo": random.randint(1000, 6000),
        "GN": 50.0518297,
        "GE": 14.5015125,
        "time": str(start_time_2)
    })

with open(path, "w+") as f:
    json.dump(data, f)
