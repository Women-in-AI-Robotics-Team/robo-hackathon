#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor ,  InertialUnit
import math
import numpy as np
from scipy.interpolate import interp1d

## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3

def point_follower(current,goal, distance_to_goal, angle_to_goal):
    #print("Current goal: ", goal)
    # Implement a Controller to reach the goal.

    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
    
    # leftSpeed is a float
    # rightSpeed is a float
    
    
    # Arc length formula
    centre_of_rotation = distance_to_goal / angle_to_goal
    print("Centre of rotation", centre_of_rotation)
   
    # TO DO ----- 
    
    # given distance to centre of rotation, determine velocities of left & right wheel
   
    
    # Controller code here to reach from "current" location to "goal" location. 
    
    
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    leftSpeed = 1.0
    rightSpeed = 1.0
    return leftSpeed,rightSpeed

def Sketch():
    # Use this function to calculate waypoints that can trace the given curve in the world. 
    # This is optional you can also impelement everything in point_follower function.
    # Clue: You can think of equations for the curve and create waypoints to follow it.
    # How to add additional goal points to the goal vector :
    # Eg: to add [1,1,0] as goal
    # goal =[]
    # goal.append([1,1,0])
    # then to add [1,2,1.57] as goal
    # goal.append([1,2,1.57])
    # To access 1st goal : goal[0], 2nd goal : goal[1] and so on..
    
    # Solution found at: https://stackoverflow.com/questions/51512197/python-equidistant-points-along-a-line-joining-set-of-points
    
    # Divide sin curve into 100 steps of equal distance along x axis
    # But distance to actually travel varies, so need to compute equidistance along the function
    
    x_vals = np.linspace(-5, 5, 100)
    y_vals = -5* np.sin((x_vals + 5)*2/math.pi)
    
    # Length of the sine curve
    # Find the difference between the x & y values between each pair of points along the curve
    # Use a^2 + b^2 = c^2 to find the shortest distance between the pairs 
    # um all values to get length of the curve
    # np.cumsum returns a list of the cumulative sum of all the distances
    # np.ediff1d finds differences between consecutive elements of an array
    
    length = np.cumsum(np.sqrt( np.ediff1d(x_vals, to_begin=0)**2 + np.ediff1d(y_vals, to_begin=0)**2 ))
    
    # Set values in the list between 0 and 1 by dividing by total length
    length = length/length[-1]
    
    #create two functions, fx and fy that builds back / approximates a function from the given points
    fx, fy = interp1d( length, x_vals ), interp1d( length, y_vals )
    
    new_steps = np.linspace(0, 1, 15)
    x_new, y_new = fx(new_steps), fy(new_steps)
    
    x_pairs = list(zip(x_vals, x_vals[1:]))
    y_pairs = list(zip(y_vals, y_vals[1:]))
    
    yaws = []
    
    for (x0, x1), (y0, y1) in zip(x_pairs, y_pairs):
        slope = (y1-y0)/(x1-x0)
        slope_angle = math.atan(slope)  # slope angle in radians
        yaws.append(slope_angle)
    
    goal = []
    
    for x, y, w in zip(x_new, y_new, yaws):
        goal.append([x, y, w])
    
    #goal = [0,0,0] # [x,y,theta]
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
    
#goal = Sketch()
#step = 0

if __name__=="__main__":

    #Optional Edit here ------------------------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------
    goals = Sketch()
    print("goals: ", goals)
    current_goal = goals[0]
    
    while robot.step(timestep) != -1:

        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]] 
        # current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        print("current x, y, theta of robot: ",current)
        
        #comment this default goal location if you caculate your own set of goals vector 
        #goal = goals # initial goal to initialize goal array

        ## ------------------------------------------------------------------------------
        #Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed 
        
        # if distance between current position and current goal is < 0.01 
        # then move goal over by 1 and set new goal
        
        print("current goal: ", current_goal)
        x_current = current[0]
        x_goal = current_goal[0]
        
        y_current = current[1]
        y_goal = current_goal[1]
        
        yaw_current = current[2]
        yaw_goal = current_goal[2]
          
        distance_to_goal = np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2)
        print("distance_to_goal", distance_to_goal)
        
        yaw_difference = yaw_current - yaw_goal
        
        # get smallest difference angle
        # sin determines deg to left or right (+ve or -ve)
        # mod makes sure we're always travelling < 180deg
        angle_to_goal = np.sign(yaw_difference) * np.mod(yaw_difference, np.pi/2)
    
        if distance_to_goal < 0.05:
            if len(goals) == 1:
                break
            goals = goals[1:]
            current_goal = goals[0]
        
        leftSpeed,rightSpeed = point_follower(current,current_goal, distance_to_goal, angle_to_goal)
    
        ## ------------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
        #step += 1
