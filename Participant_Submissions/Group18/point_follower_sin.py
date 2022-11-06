#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor ,  InertialUnit
import math
import numpy as np

## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3
reached =0
goal_reached = False
def point_follower(current,goal,leftSpeed,rightSpeed,reached,goal_length):
    # Implement a Controller to reach the goal.
    print(goal)
    global goal_reached
    x = goal[0]
    y = goal[1]
    
    inc_x = x - current[0]
    inc_y = y - current[1]
    #slope of diffeence between goal and current
    angle = math.atan2(inc_y, inc_x)
    print("angle",angle)
    print(abs(angle-current[2]))
    if abs (angle - current[2]) > 0.2 and reached<(goal_length/2)+1:
        leftSpeed = -1
        rightSpeed = 1
        print("TURN LEFT")
    elif abs (angle - current[2]) > 0.1 and reached>=(goal_length/2)+1:
        leftSpeed = 1
        rightSpeed =-1
        print("TURN RIGHT")
     
        
    elif math.dist([current[0], current[1]],[goal[0], goal[1]]) < 0.1:
            goal_reached = True
            print("Reached the goal!")        
    else:
        leftSpeed = 10
        rightSpeed = 10
        print ("moving forward")
        if reached==goal_length-1 and goal_reached:
            leftSpeed=0
            rightSpeed=0
    return leftSpeed,rightSpeed
   
    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
      
    # leftSpeed is a float
    # rightSpeed is a float

    # Controller code here to reach from "current" location to "goal" location. 
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    

def Sketch(start_position):
    # Use this function to calculate waypoints that can trace the given curve in the world. 
    # This is optional you can also impelement everything in point_follower function.
    # Clue: You can think of equations for the curve and create waypoints to follow it.
    # How to add additional goal points to the goal vector :
    # Eg: to add [1,1,0] as goal
    # goal =[]
    goal = []
    a = 5
    x=start_position[0]
    for i in np.linspace(math.pi,math.pi*3,20)[1:]:
        diff = start_position[0]-(start_position[0] *(math.pi*11/10)/(math.pi))
        x=x+diff
        y = a* math.sin(i)
        goal.append([x,y])
    # goal.append([1,1,0])
    # then to add [1,2,1.57] as goal
    # goal.append([1,2,1.57])
    # To access 1st goal : goal[0], 2nd goal : goal[1] and so on..
     # [x,y,theta]
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
   
    
    while robot.step(timestep) != -1:
           
        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        print("current x, y, theta of robot: ",current)
        
        if starting:
            start_pos = current
            leftSpeed = 10
            rightSpeed = 10
            starting = False
        goal = Sketch(start_pos)
        goal_length=len(goal)
            
        if goal_reached and reached<=17:
            reached+=1
            leftSpeed,rightSpeed = point_follower(current,goal[reached],leftSpeed,rightSpeed,reached,goal_length)
            goal_reached = False
            print(goal[reached])
            
        else:
             leftSpeed,rightSpeed = point_follower(current,goal[reached],leftSpeed,rightSpeed,reached,goal_length)
             print("usual",goal[reached]) 
        print(goal_reached,reached)
                #comment this default goal location if you caculate your own set of goals vector 
        #goal = [0,0,0] # initial goal to initialize goal array

        ## ------------------------------------------------------------------------------
        #Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed 
        ## ------------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
