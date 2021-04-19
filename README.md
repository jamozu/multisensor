# Multi-Sensor
This project consists of a module that manages different sensors for home use. The idea is to be able to mix and match different sensors, according to the requirements of each area of the house, using a single PCB design and a configurable sketch, making it easier to setup and maintain a sensor network.

The sensor module is based on the MySensors library using an Arduino® Mini Pro, which has a good number of interface pins, it has a small footprint and is suited for running on batteries, with the NRF24L01 radio module for wireless communication.

For a full description check the Doc Multi_Sensor.pdf (https://github.com/jamozu/multisensor/blob/main/Multi_Sensor.pdf).

Details on the PCB: https://oshwlab.com/jamozu/multi_project_copy_copy_copy_copy_copy_copy

# License
This project is licensed under the CERN-OHL-P v2

# Special Thanks to
MySensors Team, by creating a great and inspiring platform for all DIYers and IoT enthusiasts. (http://www.mysensors.org)
Damian and Gnd_To_Vcc, for their tips on using PIR sensors with 3.3 V (https://screenzone.eu/modify-hc-sr501-pir-sensor-to-work-with-3-3v/), (https://gndtovcc.home.blog/2020/04/23/modifying-cheap-pir-motion-sensor-to-work-at-3-3v/)
system, for an excellent template for writing values to EEPROM (https://forum.arduino.cc/t/writing-float-values-to-eeprom/41607/3)
hek, who has a very complete example of the use of air quality sensors. (https://www.mysensors.org/build/gas)
To all the DIYers out there who are enriching these IoT development platforms, making it easier for new enthusiasts.
