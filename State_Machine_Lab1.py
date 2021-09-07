# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor


TIME_STEP = 64

MAX_SPEED = 6.28


# create the Robot instance.
robot = Robot()


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# initialize devices
ps = []
psNames = [
'ps0', 'ps1', 'ps2', 'ps3',
'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    # Process sensor data here.
    
    #detect obstacles
    front_obstacle = psValues[0] > 80.0 and psValues[7] > 80.0
    
    
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    # modify speeds according to obstacles
    if front_obstacle:
        # turn 180 
        while psValues[3] < 80.0 and psValues[4] < 80.0:
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
    
    
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

