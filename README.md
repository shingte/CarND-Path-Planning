# CarND-Path-Planning-Project

Path planning of a car around a simulated highway using traffic, waypoint and sensor fusion data. The car must not violate maximum velocity, acceleration, and jerk, while also avoiding collisions with other vehicles, keeping to within a highway lane.
   
[![Path-Planning](./ref/Path_Planning.gif)](https://youtu.be/QpL-A0ebQG0)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

The car's localization and sensor fusion data are provided, there is also a sparse map list of waypoints around the highway. 

The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 

The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 

The car should be able to make one complete loop around the 6946m highway. 

Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 

Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


## Reflection

Based on the provided code from the seed project, the path planning algorithms start at src/main.cpp line 272 to line 509. 

The code consist of three parts:

1. Prediction ([line 272 to line 335](./src/main.cpp#L272))

    This part of the code deal with the telemetry and sensor fusion data. In the case, we want to know three aspects of it:

    Is there a car in front of us blocking the traffic.
    
    Is there a car to the right of us making a lane change not safe.
    
    Is there a car to the left of us making a lane change not safe.
    
    These questions are answered by calculating the lane each other car is and the position it will be at the end of the last plan trajectory. 
    
    The flags is_car_ahead, is_car_right, is_car_left are used to indicate if there are cars ahead of at left/right lanes. The distance and velocity of the car ahead are also recorded.

2. Behavior ([line 337 to line 408](./src/main.cpp#L337))

    This part decides what to do:

    If we have a car in front of us, do we change lanes?
    
    Do we speed up or slow down?
    
    Based on the prediction of the situation we are in, this code increases the speed, decrease speed, or make a lane change when it is safe. 
    
    It will try to make a lane change if a car is ahead and within the safty range. 
    If not, it will speed up or slow down based on the distance and relative velocity of the car in front of the ego car.

3. Trajectory ([line 411 to line 509](./src/main.cpp#L411))

    Calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points using spline fitting.

    First, the last two points of the previous trajectory (or the car position if there are no previous trajectory, lines 422 to 430) are used in conjunction three points at a far distance (lines 446 to 448) to initialize the spline calculation. 
    
    To make the work less complicated to the spline calculation based on those points, the coordinates are transformed to local car coordinates (lines 458 to 464).

    The rest of the points are calculated by evaluating the spline and transforming the output coordinates to global coordinates. We always output 50 points.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```






