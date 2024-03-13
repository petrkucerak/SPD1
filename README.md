# Weather station with geolocation

Authors: Petr Kucera, Lukas Nejezchleb


## Project description

### Goal and motivation

If you go for cold water swimming, it might be interesting for you to know the parameters of the particular river, stream, or pond. But how do you manage when the Czech Hydrometeorological Institute does not monitor the given watercourse? What if you could keep a diary that would not only collect the water and air temperature but also allow you to record how you felt on a particular day after the cold swimming?

The aim of this project is to create weather station capable of measuring temperature, humidity, water temperature, light. In addition, the station will be equipped with GPS sensor, so that the user can distinguish between data from different stations using positions in the map. 

### Use case
Various weather station projects are currently available. When we consider the requirement to have multiple weather stations (there can be quite different weather conditions in locations close-by, especially in mountainous terrain). Our approach, which uses GPS positioning to distinguish between different nodes, could help with that.

### Project architecture

For simplicity, let's divide the project into two environments: *river environment* and *house environment*.

![project structure](/assets/structure.png)

The **river environment** includes a device that is placed on the riverbank. One sensor, measuring water temperature, is located in the water, while other sensors measuring temperature and humidity, position (GPS), and light are placed above the water. The device operates hourly and is powered by batteries. It sends data several times a day using LoRa technology. Code will be implemented in C/C++.

In the **house environment**, there is a device called the *concentrator*. It receives data from devices by the watercourse using LoRa technology and stores it in a database. The device is connected to the home network via RJ-45 or WiFi. It runs a web server that displays the measured results on a map. Handle routines will be implemented in C/C++/Python and server in JavaScript/Typescript.


## Requirements
To implement a weather station node, the following material is required
- STM32 Nucleo pack - L073RZ + LoRa SX1272 (*2x for parallel development*)
- GPS L86 - GNSS click
- SHT31-D Digital temperature & humidity sensor
- Grove - Light Sensor v1.2
- DS18B20 Waterproof Digital temp sensor 

For master bus collecting data from nodes
- Raspberry Pi 3 B + LoRa shield + 32GB SD
