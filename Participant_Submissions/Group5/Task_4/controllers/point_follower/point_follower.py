#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import Keyboard , GPS , Motor ,  InertialUnit
import math

## ------------------------------------------------------------------------
# Edit keyboard_follower function to control robot using your choice of keyboard keys 

def setVelocity(leftSpeed, rightSpeed):
    wheels[0].setVelocity(leftSpeed) # Front left wheel
    wheels[1].setVelocity(rightSpeed) # Front right wheel
    wheels[2].setVelocity(leftSpeed) # Rear left wheel
    wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
def makecircle():
    start = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]] 
    setVelocity(5,3)
    robot.step(timestep)

    current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]  
    inc = 0  
        
    while robot.step(timestep) != -1 and inc < 500:

        # x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        # print("current x, y, theta of robot: ",current)
       
        setVelocity(5,3)
        inc= inc + 1
        
        ## ------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
    setVelocity(0,0)
    
    
def makeline():
    start = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]] 
    setVelocity(3,3)
    robot.step(timestep)

    current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]  
    inc = 0  
        
    while robot.step(timestep) != -1 and inc < 300:

        # x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        # print("current x, y, theta of robot: ",current)
       
        setVelocity(3,3)
        inc= inc + 1
        
        ## ------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
    setVelocity(0,0)
        

def keyboard_follower(key, lastkey, current):
    # Assign different direction of motions with different key values.

    # key is an integer value
    # leftSpeed is a float
    # rightSpeed is a float
    
    leftSpeed = 0
    rightSpeed = 0
    
    if(key != -1):
        print(key)
    
    # if paint == True: 
    # if key == lastkey:
        # leftSpeed = 0
        # rightSpeed = 0
    if key == 315:
        leftSpeed = 5.0
        rightSpeed = 5.0
        print("up")
    elif key == 317:
        leftSpeed = -5.0
        rightSpeed = -5.0
        print("down")
    elif key == 316:
        leftSpeed = 5.0
        rightSpeed = -5.0
        print("right")
    elif key == 314:
        leftSpeed = -5.0
        rightSpeed = 5.0
     
        print("left")
    elif key == 76:
        makeline()
    elif key == 67:
        makecircle()
        print("circle")
           

    # leftSpeed = 0
    # rightSpeed = 0
    # Sample velocities provided to make robot move straight. 
    return leftSpeed,rightSpeed

# Edit point_follower and Sketch functions similar to Task 1/2/3


## ------------------------------------------------------------------------

#Initializing robot to access sensor data from the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # Defined timestep (in msec) to enable sensors and define sensor frequency.

# Initialize and Enable Keyboard object to get X,Y location of robot
keyboard=Keyboard()
keyboard.enable(timestep)

# Initialize and Enable GPS object to get X,Y location of robot
gps = robot.getGPS("gps")
gps.enable(timestep) # x, y, z location received at time difference equat to timestep.

# Initialize and Enable IMU object to get theta (orientation) of robot
imu = robot.getInertialUnit("imu")
imu.enable(timestep) # Theta recieved at time difference equat to timestep.

# Initialize and Enable robot wheels
wheels_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = []
for i in range(4):
    wheels.append(robot.getMotor(wheels_names[i]))
    wheels[-1].setPosition(float('inf')) # setting max position limit of each wheel to infinity to convert it to velocity mode 
    wheels[-1].setVelocity(0.0) # Setting zero velocity for all the wheels.

if __name__=="__main__":

    #Optional Edit here if using Option 2 ------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------

    lastkey = -1
    while robot.step(timestep) != -1:

        # x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        # print("current x, y, theta of robot: ",current)
        goal = [0,0,0] # initial goal to initialize goal array

        ## ------------------------------------------------------------------------
        #Edit here to control using Keyboard
        # Use keyboard_follower function to assign different direction of motions with different key values.
        # keyboard_follower should return leftSpeed and rightSpeed 
        key=keyboard.getKey() # getting currently pressed key on the keyboard.
        #print("Key pressed: ", key)
        leftSpeed,rightSpeed = keyboard_follower(key, lastkey, current)
        lastkey = key
        
        ## ------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
