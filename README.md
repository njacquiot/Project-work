# Thesis
The following repository contains the source code to the implemented prototypes, which are based on a RPi with a Dragino LoRa GPS HAT.
## Receiver
* The implementation in the [LoRa](https://github.com/christofferbrask/Thesis/tree/master/Receiver/LoRa) folder is able to run all the different test modes. GPS data is logged with the measured values. A SSD1306 oled display provides real-time measurements.
* The implementation in the [SF6](https://github.com/christofferbrask/Thesis/tree/master/Receiver/SF6) folder is able to run the different test modes with a spreading factor of 6.
* The implementation of the [TTN node](https://github.com/christofferbrask/Thesis/tree/master/Receiver/TTN%20node) is a cloned project from the following GitHub [page](https://github.com/ernstdevreede/lmic_pi/). The TTN keys, identifiers and device address are changed to work with a TTN application. The spreading factor and frequency are changed when the different tests are conducted. 
## Sender
* The implementation in the [LoRa](https://github.com/christofferbrask/Thesis/tree/master/Sender/LoRa) folder is able to run the equivalent test modes to the receiver. The sender does not generate any log files with measurments, but is used to provide the different data that are required on the receiver. The sender is also able to respond on the ping message used in the round trip time test.
* The implementation in the [SF6](https://github.com/christofferbrask/Thesis/tree/master/Sender/SF6) folder is able to run the run the equivalent test modes to the receiver with a spreading factor of 6.
* The only difference in the implementation of the [TTN node](https://github.com/christofferbrask/Thesis/tree/master/Sender/TTN%20node) is the device address. 
## Simulator
* The folder contains the python script that is used to simulate the behavior of a drone in Mission Planner.
## Links
[[1]](https://youtu.be/Kvb3IbS0xYI) Video - Simulation of drone, that runs a MAVLink script. <br />
[[2]](https://youtu.be/-3Y6np6e9lA) Video - Drone control. <br />
[[3]](https://bit.ly/2LzhaC5) Google Maps - In-air operation test. <br />
