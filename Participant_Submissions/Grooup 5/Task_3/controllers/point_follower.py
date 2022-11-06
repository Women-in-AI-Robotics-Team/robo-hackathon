#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor ,  InertialUnit
import math
from decimal import Decimal

## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3

def point_follower(current,goal,angle, t):
    # Implement a Controller to reach the goal.

    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
      
    # leftSpeed is a float
    # rightSpeed is a float

    # Controller code here to reach from "current" location to "goal" location. 
    inc = t
    diff = angle - current[2]
    print("diff", diff)
    
    # if (abs(goal[0]-current[0]) <= 0.1 and abs(goal[1]-current[1]) <= 0.1):
        # inc = inc + 1
    if (abs(diff) <= 0.1):
        leftSpeed = 5
        rightSpeed = 5
        if abs(current[0] - goal[0]) < 0.2 and abs(current[1] - goal[1]) < 0.2: 
            inc = inc + 1
            
            
    else: 
        if diff < 0:
            #move right
            leftSpeed = 1
            rightSpeed = -1
        else:
            leftSpeed = -1
            rightSpeed = 1
    
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    return inc,leftSpeed,rightSpeed

def Sketch(t):
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
    goal = []
    while (t >=0):
        goal.append([(t/10)*math.cos(-t)-0.03, (t/-10)*math.sin(t+0.0069), 0])
        t = t - 0.1
    return goal

def move():
    left = 1
    right = 1
    setVelocity(left, right)

def setVelocity(leftSpeed, rightSpeed):
    # Setting velocities to each wheel based on calculation.
    wheels[0].setVelocity(leftSpeed) # Front left wheel
    wheels[1].setVelocity(rightSpeed) # Front right wheel
    wheels[2].setVelocity(leftSpeed) # Rear left wheel
    wheels[3].setVelocity(rightSpeed) # Rear right wheel
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
    move()
    t = 48.79
    # t = 10
    goals = Sketch(t)
    # for x in goals:
        # print("(",x[0], "," ,x[1],")")
    index = 0
    while robot.step(timestep) != -1 and index <= 487:
        # break
        print("index: ", index)

        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        print("current: ",current)
        
        #comment this default goal location if you caculate your own set of goals vector 
        goal = goals[index]
        # slope = ((-10)/math.pi) * math.cos(2 * (current[0] + 4.95) / math.pi)
        diffx = goal[0] - current[0]
        diffy = goal[1] - current[1]
        # slope = diffy/diffx
        angle = math.atan2(diffy, diffx)
        print("goal: ", goal)
        print("diffx: ", diffx, " diffy: ", diffy)
        # print("slope: ", slope)

        # angle = math.atan(slope)
        # angle = degree * math.pi / 180
        # print("diffx: ", diffx, "diffy: ", diffy)
        print("angle: ", angle)
        
        ## ------------------------------------------------------------------------------
        #Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed 
        index,leftSpeed,rightSpeed = point_follower(current,goal,angle,index)
        ## ------------------------------------------------------------------------------

        setVelocity(leftSpeed, rightSpeed)
        print("-------------------------------------------")
