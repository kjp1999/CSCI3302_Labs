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
hFirst = False
Right = False

def Right():
    while robot.step(TIME_STEP) != -1:
        #set wheel speeds oposite to turn   
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        
        #read sensor values
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
        #if sensor 5 reads 125 set wheel speeds the same to go straight and return
        if psValues[5] >= 125:
            leftSpeed = .5 * MAX_SPEED
            rightSpeed = .5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            return
        
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

def Around():
    while robot.step(TIME_STEP) != -1:
            
        #read sensor values
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
        
        if psValues[3] >= 125 and psValues[4] >= 125:
            leftMotor.setVelocity(.5 * MAX_SPEED)
            rightMotor.setVelocity(.5 * MAX_SPEED)
            return
            
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(-0.5 * MAX_SPEED)
        


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    f_obstacle = psValues[0] >= 100 or psValues[7] >= 100
    
    if Right == True:
        # print(psValues[5])
        if psValues[5] < 100:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    
    # modify speeds according to obstacles
    if f_obstacle:
        if hFirst == False:
            # turn 180 degress clockwise
            Around()
            hFirst = True
        elif hFirst == True:
            Right()
            Right = True
        
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)