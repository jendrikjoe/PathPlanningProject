# CarND-Path-Planning-Project
This is my solution to the Udacity Self-Driving-Car Nanodegree.
The markup addresses the rubric points at https://review.udacity.com/#!/rubrics/1020/view

## The code compiles correctly.

It does :)

## Valid Trajectories 

- The car is able to drive at least 4.32 miles without incident, when not taking the simulator bug into account, which is throwing an out of line, when driving in the outmost right lane at a certain pint in the simulator.
- The car drives according to the speed limit The car is driving around 48 mph most of the time. Exceptions are the cases, where another car is blocking the lane and a changing of the lanes is not possible.
- The maximum acceleration and jerk are not exceeded. 
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes

## Reflection

The generation of the trajectories in this project is done in the car's local reference frame and not in Frenet coordinates as those lead to warping when driving in the most right line (thanks to John Chen for pointing this out in Slack. 
The overall workflow contains the following steps:
- The BehaviourPlanner object gets the car's coordinates, the sensor fusion data sensing all the other cars and the current speed. Based on these data it is deciding in what lane the car shall drive and with which speed it shall do that. 
- The TrajectoryGenerator object is fed with the lane the car shall drive in, the previous path (if existent), the car's coordinates and the current speed and is deriving a drivable path from these parameters.
- The SpeedController object the uses the created path, the speed command, the current speed, the car's coordinates and the previous path (if existent) and samples a drivable version of the path, which is respecting the speed, acceleration and jerk constraints given.

### The Behaviour Planner

#### The Workflow
The BehaviourPlanner object first converts the sensor fusion data into the car's local reference frame. Then is splits them into the individual lanes. Here it applies the following rules:
- $d<3$: Add the data only to the left lane
- $3<d<5$: Add the data to the left lane and middle lane, as the car is changing lanes
- $5<d<7$: Add the data only to the middle lane
- $7<d<9$: Add the data to the middle lane and right lane, as the car is changing lanes
- else: Add the data only to the right lane

Now it is checked, if the lane was changed in the last three seconds, if so only the speed is controlled, otherwise it is checked if the lane shall be changed. This avoids excessive lane changing.

If a lane decision shall be made the planner calculates the cost for each of the lanes. The cost factors are the following:
- If there is a car 5 meter behind our car changing is impossible to avoid collisions.
- The same applies if a car is 10 meter behind us and more than 1 m/s faster than us.
- If a car is less than 10 meter in front of our car then a lane change is impossible as well. 
- If the lanes is different from the current one there is an extra penalty of 10, to avoid unnecessary lane changes.
- The distance between the current and the other lane is added to penalise crossing a lane.
- If the distance to the next car in front is smaller than 100m, the difference to 100m is added as a cost factor as well.
- If the distance to the next car in front is smaller than 100m, the difference from its speed to the maximum speed times 10 is as well added to the cost function, if it is slower than the maximum speed.


With these costs for each lane, the planner is determining the cheapest feasible lane, taking into account, that, if a lane is crossed, the crossed lane has to be free. The lane in-between is considered free, if:
- There is no car within less than 25m in front and more than 5m/s slower than our car in the middle lane.
- There is no car within less than 15m behind and more than 2m/s faster than our car in the middle lane.
- There is no car within less than 10m in front of our car in the middle lane.
- There is no car within less than 10m beind our car in the middle lane.

If the lane is decided upon, the planner decides about the speed. If a lane is currently crossed the speed is:
- the maximum speed if there is no car within 25m in front of us
- the speed of the car in front of us + 2m/s if the car is more than 15m in front of ours
- the speed of the car in front of us - 2m/s if the car is more than 5m in front of ours
- the speed of the car in front of us - 5m/s if it is closer than 5m from us.

If no lane is crossed, the rules are the following:
- If the distance to the car in front of us is less than 5m, the speed of the car in front of us - 5m/s
- If the distance to the car in front of us is less than 10m, the speed of the car in front of us - 2m/s
- If the distance to the car in front of us is less than 20m, the speed of the car in front of us + 0.5m/s
- If the distance to the car in front of us is less than 30m, the speed of the car in front of us + 2m/s
- maximum speed otherwise.

The line and speed command are then returned and later send to the Trajectory Generator.

#### Possible Improvements
Possible Improvements would be:
- Add the capability to decelerate to fall behind cars in other lanes. This could especially solve the case where the middle lane is block, but either the right or the left are completely free, but the car cannot cross the middle lane.
- Let a neural network with reinforcement learning decide upon the lane changes and speed of the car. This would allow to avoid complex rule definition.

### The Trajectory Generator

#### The Workflow
The TrajectoryGenerator object first determines the target d value from the selected lane. Then it determines the direction in which the car has to change its current d value. With this direction information it determines a new target d to make sure, that the side acceleration is not exceeded. The maximum d change is hereby set to currentSpeed/8, where 8 is found using a trial and error based approach.
Then the generator is building two splines, one representing the current trajectory of the car in local coordinates and the other one representing the trajectory for the target d value. 
Then a point distance of a twentyth of the previous path total x-length is set.
With this point distance a sampling is done, taking 10 samples in x-direction from the previous path, skipping thrity samples and the taking ten samples from the target trajectory. With these sampled points a new spline is made, giving a relatively smooth transition from the previous path to the target path.
This spline is then giving the path whcih shall be executed by the car.

If no previous trajectory is available, only the target path's spline is returned as the target path.

#### Possible Improvements

The trial and error determined value could be replaced with a mathematically determined value, or one which is actually checking the jerk and acceleration caused by a change in d.

### The Speed Controller

The SpeedController object builds a speed spline by sampling the transition from the current Speed to a final speed, which is the minimum of the targetSpeed and the currentSpeed + length of the target path. This avoids excessive acceleration and jerk.
Then, the previous path is taken and the first 12 x-coordinates from it are sampled. For these x-Values the y-values are determined by transitioning from the previos path's y-values to the target path's spline values using a sigmoid weighting function. Afterwards, 38 points are added, by extracting the speed from the speed spline using distance already covered. With this speed the new x is added, by taking the last distance and adding the speed*stepWidth to it. The y-Value is then extracted from the target path spline and added to the trajectory. 

#### Possible improvements

The controller could take the heading into account to only add the speed to the x-Coordinate, which is actually covered in x-Direction. This would lead to a less noisy speed and less acceleration.






















