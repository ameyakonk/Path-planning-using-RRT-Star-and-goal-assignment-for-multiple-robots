# Path-planning-using-RRT-Star-and-goal-assignment-for-multiple-robots

## Overview
The repository contains the following contents.

1. ROS Package and source code to run the RRT* path planner and goal assignment solver.
2. Video files suggesting node exploration and finding optimal path to goal in Gazebo and Python

We have used three different start and end points for 3 different robots:

a. start1 = (50, 50)
   start2 = (400, 100)`
   start3 = (900, 100)
  

b. goal1 = (900, 900)
   goal2 = (500, 950)
   goal3 = (900, 650)

## Personnel
### Ameya Konkar 

UID:118191058

Master's Student at University of Maryland, College Park

### Ninad Harishchandrakar

UID:118150819

Master's Student at University of Maryland, College Park

## Dependencies 

1. Python 3
2. Numpy
3. Matplotlib
4. ROS
5. Gazebo
6. Scipy

### Building the Program and Tests

```
sudo apt-get install git
git clone --recursive https://github.com/ameyakonk/Path-planning-using-RRT-Star-and-goal-assignment-for-multiple-robots.git
```
To Run the code:
```
cd catkin workspace/ 
catkin build
roslaunch Final_Project_661 turtlebot3_autorace.launch

```
