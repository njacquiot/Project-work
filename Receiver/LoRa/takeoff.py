from dronekit import connect, VehicleMode
import time
import sys
from pymavlink import mavutil # Needed for command message definitions

# ---------------------- YAW-control (turn) script ------------------------

def turn(heading, relative, direction):
    print "Current Heading: %s" % vehicle.heading
        
    if relative:
        is_relative = 1 #yaw relative to direction of travel
        print "Turning", direction, heading, "degrees relative to current heading."
        if direction == "CW":
            newHeading = vehicle.heading+heading
            direction = 1
        else: 
            newHeading = vehicle.heading-heading
            direction = -1
        if newHeading>360:
            newHeading-360
        if newHeading<0:
            newHeading+360
        print "New Heading: %s" % newHeading
    else:
        is_relative = 0 #yaw is an absolute angle
        print "Turning to %s degrees absolute." % heading
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    while True:
        print "Current heading: %s" % vehicle.heading
        if direction == 1 and vehicle.heading>=newHeading:
            print "New heading reached"
            break
        elif direction == -1 and vehicle.heading<=newHeading:
            print "New heading reached"
            break
        time.sleep(0.5)

# ---------------------- ARM and TAKEOFF script ------------------------

def arm_and_takeoff(aTargetAltitude):
    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    #while not vehicle.is_armable:
    #    print " Waiting for vehicle to initialise..."
    #    time.sleep(1)

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

# ---------------------- Start of program ------------------------

#vehicle = connect('udp:127.0.0.1:14550',wait_ready=True)
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=57600)
#vehicle = connect("/dev/ttyACM0", baud=115200)
print "Connected to vehicle"

print vehicle.battery
print vehicle.location.global_frame

# Arm and take off to a height determined by the user (a ground station)
arm_and_takeoff((vehicle.location.global_relative_frame.alt+int(sys.argv[1])))
       
#time.sleep(3)
#turn(180,True,"CW")
#time.sleep(3)
#turn(180,True,"CCW")
#time.sleep(3)
#vehicle.mode = VehicleMode("LAND")
# Close vehicle object and disarm
#vehicle.mode = VehicleMode("STABILIZE")
#vehicle.armed  = False