#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor ,  InertialUnit
import math
from math import radians


is_end = False
## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3



def point_follower(current,goal,start,l,r):
    global is_turning_first
    global is_turning_second
    global is_turning_third
    global is_end
    #global is_end
    # Implement a Controller to reach the goal.

    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
      
    # leftSpeed is a float
    # rightSpeed is a float

    # Controller code here to reach from "current" location to "goal" location. 


### Checking angle if the robot should stop turning and start going straight

    if math.dist([current[2]], [goal[1][2]]) <= 0.005 and is_turning_first:
        is_turning_first = False
        return 10,10
        
    if math.dist([current[2]], [goal[2][2]]) <= 0.005 and is_turning_second:
        is_turning_second = False
        return 10,10
    
    if math.dist([current[2]], [goal[3][2]]) <= 0.007 and is_turning_third:
        is_turning_third = False
        return 10,10
 
       
### Checking if the the robot reached the goal
     
    if math.dist([current[0], current[1]],[start[0], start[1]]) < 0.5:
        leftSpeed = 10
        rightSpeed = 10
        
        if math.dist([current[0], current[1]],[goal[0][0], goal[0][1]]) < 0.1 and is_end:
            leftSpeed = 0
            rightSpeed = 0

    elif math.dist([current[0], current[1]],[goal[1][0], goal[1][1]]) < 0.1:
        leftSpeed = -1
        rightSpeed = 1
        is_turning_first=True
        
    elif math.dist([current[0], current[1]],[goal[2][0], goal[2][1]]) < 0.1:
        leftSpeed = -1
        rightSpeed = 1
        is_turning_second = True

    elif math.dist([current[0], current[1]],[goal[3][0], goal[3][1]]) < 0.1:
        leftSpeed = -1
        rightSpeed = 1
        is_turning_third = True
        is_end = True
     
    else:
        leftSpeed = 10
        rightSpeed = 10
             
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    return leftSpeed,rightSpeed

def Sketch(start):
    # Use this function to calculate waypoints that can trace the given curve in the world. 
    # This is optional you can also impelement everything in point_follower function.
    # Clue: You can think of equations for the curve and create waypoints to follow it.
    # How to add additional goal points to the goal vector :
    # Eg: to add [1,1,0] as goal
    goal =[]
    # goal.append([1,1,0])
    # then to add [1,2,1.57] as goal
    # goal.append([1,2,1.57])
    # To access 1st goal : goal[0], 2nd goal : goal[1] and so on..
    goal.append([-4, -4, 0])
    goal.append([4, -4, radians(90)])
    goal.append([4, 4, radians(180)])
    goal.append([-4, 4, radians(-90)])
    return goal

## ------------------------------------------------------------------------

#Initializing robot to access sensor data from the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # Defined timestep (in msec) to enable sensors and define sensor frequency.

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

    #Optional Edit here ------------------------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------
    
    starting = True
    leftSpeed = 0
    rightSpeed = 0
    
    while robot.step(timestep) != -1:

        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        
        
        if starting:
            start_pos = current
            leftSpeed = 10
            rightSpeed = 10
            starting = False
        
        #comment this default goal location if you caculate your own set of goals vector 
        goal = Sketch(start_pos)

        ## ------------------------------------------------------------------------------
        #Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed 
    
        leftSpeed,rightSpeed = point_follower(current,goal,start_pos,leftSpeed,rightSpeed)
        ## ------------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
