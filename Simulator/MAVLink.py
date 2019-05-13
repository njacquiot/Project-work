from dronekit import VehicleMode, LocationGlobalRelative, LocationGlobal
import dronekit_sitl, time
import math
from pymavlink import mavutil

# Import DroneKit-Python
from dronekit import connect, VehicleMode

def turn(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

# ---------------------- ARM and TAKEOFF script ------------------------

def arm_and_takeoff(aTargetAltitude):
    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

# ----------------- goto position ----------------------

def goto_position(location):
    targetDistance=get_distance_metres(vehicle.location.global_frame, location_200m)
    
    vehicle.simple_goto(location)

    while True:
        remainingDistance=get_distance_metres(vehicle.location.global_frame, location)
        print "Distance to target: %.1f" % remainingDistance
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print "Reached target"
            break
        time.sleep(2)


# ---------------- distance to current waypoint ---------------

def get_distance_metres(location1, location2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlon = (location2.lon-location1.lon)*math.pi/180
    dlat = (location2.lat-location1.lat)*math.pi/180
    a = math.sin(dlat/2)**2+math.cos(location1.lat*math.pi/180)*math.cos(location2.lat*math.pi/180)*math.sin(dlon/2)**2

    c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
    # Radius of earth in m
    R = 6371000

    return R*c

# ---------------------- Start of program ------------------------
# Connect to the Vehicle. Change port to 14550 on real device
vehicle = connect('udp:127.0.0.1:14551',wait_ready=True)

print "Connected to vehicle"

print vehicle.battery
print vehicle.location.global_frame

arm_and_takeoff(vehicle.location.global_relative_frame.alt+3) # Fly up 3 meter

#vehicle.groundspeed = 10 #m/s

location_10m = LocationGlobalRelative(55.472058, 10.414303, 3)
location_20m = LocationGlobalRelative(55.472069, 10.414467, 3)
location_30m = LocationGlobalRelative(55.472069, 10.414634, 3)
location_40m = LocationGlobalRelative(55.472069, 10.414790, 3)
location_50m = LocationGlobalRelative(55.472069, 10.414951, 3)
location_60m = LocationGlobalRelative(55.472073, 10.415099, 3) 
location_70m = LocationGlobalRelative(55.472079, 10.415254, 3) 
location_80m = LocationGlobalRelative(55.472076, 10.415421, 3)
location_90m = LocationGlobalRelative(55.472073, 10.415583, 3) 
location_100m = LocationGlobalRelative(55.472080, 10.415733, 3) 
location_200m = LocationGlobalRelative(55.472092, 10.417327, 3)

goto_position(location_200m)
time.sleep(30)

#print "Return to launch"
#vehicle.mode = VehicleMode("RTL")
#Close vehicle object and disarm
#vehicle.mode = VehicleMode("STABILIZE")
#vehicle.armed  = False
#vehicle.close()
