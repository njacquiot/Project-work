# Simulator
The following python script can be used to simulate a drone that fly to a given position using MAVLink.
### Prerequisites
Mission planner is required and can be downloaded from Ardupilot's [website](http://ardupilot.org/planner/docs/mission-planner-installation.html). <br/>

Dronekit needs to be installed:
```python
pip install dronekit
pip install dronekit-sitl
```
### To run
```
python dronekit-sitl copter --home=55.472063,10.414156,0,90
python mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14550
python MAVLink.py
```
The start coordinates (home=....) can be changed to anywhere in the world

