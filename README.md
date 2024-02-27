| **Due Date**  | **Wednesday, Feb 28th at 1:00PM EST**                                     |
|---------------|----------------------------------------------------------------------------|
|  **Submission**   | `log.npz`on [Gradescope](https://gradescope.com/) |

# Wall Following in Simulation

## Introduction

In this lab, you will be implementing a wall follower on a simulated version of the racecar. Your goal is to make an autonomous controller that drives the racecar forwards while maintaining a constant distance from a wall on either its left or right (chosen on the fly). It should also be robust to uneven surfaces and small errors in the LIDAR data, and it should be able to recover from small deviations from the desired state; being too far, too close, or too angled.

This lab is to be done individually. In the next lab, you will join your team to get your wall follower working on the real racecar. You will be reusing the code you write for this lab so make sure it is clean and presentable to your teammates!

We have made a series of tests to evaluate the performance of your wall follower. In order to test your code properly you must start out with the template starter code. The template is still very sparse and you have plenty of freedom to implement any algorithm you'd like so long as the inputs and outputs are the same.

## Submission

After running the automated tests described at the [end of this lab](https://github.com/mit-rss/wall_follower_sim#running-the-tests), you will generate a file called ```log.npz```. **You must upload this to Gradescope for credit**. 

The lab is due on **Wednesday, Feb 28th, 2024**, 8 days from the release date.

## Download this Repository

Clone this repository into your colcon workspace:

    cd ~/racecar_ws/src
    git clone https://github.com/mit-rss/wall_follower_sim.git

Then rebuild your workspace with `colcon build`:

    cd ~/racecar_ws
    colcon build --symlink-install
    source install/setup.bash

##  Simulator

First, launch the [racecar simulator](https://github.com/mit-racecar/racecar_simulator) by running:

    ros2 launch racecar_simulator simulate.launch.xml
    
To see the simulated car, you will need to open [`rviz`](http://wiki.ros.org/rviz) through the right click menu on docker.

If you're using the [racecar docker image](https://github.com/mit-racecar/racecar_docker), Rviz will already be configured to visualize the simulator. But if not, in the left panel on the bottom click the "Add" button, and then in the "By display type" tab click "RobotModel". You should see a small blue car appear. Then click "Add" again and in the "By topic" tab click add the "/map" topic.  Repeat once more to add the laser scan topic. Under the dropdown menu of your LaserScan there should be a field called "Size (m)". Change this to 0.1 so you can see the laser scan more clearly. The checkboxes turn on and off display types, which may be useful as you add topics to visualize.

![Add button](https://i.imgur.com/85tY4tZ.png)

You should see a car in a map (walls are black, empty space is grey) and points on that map representing the points hit by the car's lidar.

![The racecar in the starting position](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_1.png)

You can change the position of the robot by clicking the "2D Pose Estimate" button on top of rviz and placing the somewhere on the map.

*Future note: You can also move the car around by plugging in a joystick into your computer*


**Note that the simulator does not include collision detection, but we will check that your car has not crashed.**

## Autonomous driving

In order to make the car drive autonomously you will need to publish messages of type [`AckermannDriveStamped`](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) to the `/drive` topic.
    
Import the `AckermannDriveStamped` type like this in your `wall_follower.py` file:

    from ackermann_msgs.msg import AckermannDriveStamped
    
## LIDAR

The racecar (and its simulation) are equipped a with LIDAR sensor that measures the distance from the racecar to its surroundings with high accuracy. All of the LIDAR data is published to the `/scan` topic.

The data is of type [`LaserScan`](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html). You can import the type in python using:

    from sensor_msgs.msg import LaserScan

The `ranges` data entry in the `LaserScan` message  is an array of the distances from the lidar sensor to the nearest obstacle. The measurements are taken at regular intervals, `angle_increment`, from the angle `angle_min` to the angle `angle_max`.

The rainbow points in this image below are the laser scan as visualized in ```rviz```. The color simply corresponds to the intensity of the scan. In the simulator this is simply the distance, but on the actual lidar it gives you an indication of how reflective the object you are scanning is. Note that there is no data in the quadrant behind the car because the LIDAR sensor does not scan the full 360 degree range.

![The racecar in a cubicle](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_2.png)

## Simulator Parameters

The simulator uses a number of parameters that might be useful for your wall follower including:

- `max_speed`: 4 meters/second
- `max_steering_angle`: 0.34 radians

A complete list can be found in the simulator's [params.yaml](https://github.com/mit-racecar/racecar_simulator/blob/master/params.yaml).
Don't change these parameters, they will be automatically reset to defaults when you run the automated test suite anyways.

## Steps to Success
How you implement the wall follower is entirely up to you. However implementing the following may get you started in the right direction:

* __Set up ROS structure__: Set up your wall follower node so that it subscribes to laser messages and publishes steering commands. Make sure you can at least make the racecar move fowards at a constant speed and turning angle before working on your controller.
* __Slice up the scan__: Consider slicing the ```ranges``` data into more useful pieces. A majority of the data won’t be useful to you if you only care about a wall to one side. When you can, try to use [```numpy```](https://numpy.org/) operations rather than for loops in your code. [Multidimensional slicing](https://docs.scipy.org/doc/numpy-1.13.0/reference/arrays.indexing.html) and [broadcasting](https://docs.scipy.org/doc/numpy-1.13.0/user/basics.broadcasting.html) can make your code cleaner and much more efficient. You can also turn any array into a ```numpy``` array with [```np.array```](https://docs.scipy.org/doc/numpy-1.13.0/reference/generated/numpy.array.html).
* __Find the wall__: There are many ways to detect a wall in a laser scan. In a perfect world, you might be able to detect it using 2 samples of the LIDAR data. However with noisy data and uneven surfaces this might not be enough. A [least squares regression](https://en.wikipedia.org/wiki/Simple_linear_regression) is an easy way to account for more noise. The [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm can “upgrade” an existing model (like least squares) to be more robust to outliers. _Note: Attempt RANSAC only if you've already built a functional wall follower. It is probably overkill._
* __Use PD or PID control__: A robust wall follower algorithm that can handle wavy wall contours and corners should probably use some sort of [PD or PID control](https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation). Simple P (proportional) control is often not enough to create a responsive and stable system. Tuning the constants of this system can be done through empirical observations or more [systematically](https://www.crossco.com/resources/technical/how-to-tune-pid-loops/).
* __Use the visusalization code__: We provided an example Python script in `wall_follower` that plots a line in Rviz. You can write something similar to this in order to make sure your code (e.g. wall detection) is working!

## Starter Code

Since we are providing you with a series of automated tests for your wall follower, we ask that you use the starter code contained in this repo and do not tweak its original structure.

Make sure that your wall follower lives in the ROS node initialized in the Python script at:

    wall_follower/wall_follower.py

However, if you want to add more Python files to keep your code organized, feel free to do so.
    
You must use the following ROS parameters in your follower:

* `desired_distance`: The distance, in meters, that the racecar should maintain from the wall
* `velocity`: The speed the racecar should move in meters per second.
* `side`: The side the wall is following represented as an integer. +1 represents the left wall and -1 represents the right wall. We chose this convention because typically we will assume the car is pointing in the positive _x_ direction. That means the left side of the car will point to the positive _y_ axis and the right side will point to the negative _y_ axis.  

The test cases will vary these parameters, so you must use them to pass.
The default values of these parameters are stored in the `params.yaml` file.
        
To launch the wall follower, run:

    ros2 launch wall_follower wall_follower.launch.xml

## Running the Tests


The git has been updated with the autograder tests. Like last lab you will be asked to run your tests locally and then submit the log.npz files that the tests generate. We have added two launch files to launch the correct simulator and tests (note: to add these files to your ros workspace see the changes to setup.py). 

To launch the new simulator, run:

    ros2 launch wall_follower launch_test_sim.launch.py

The tester code is  in `/wall_follower_sim/wall_follower/test_wall_follower.py` 

To launch the tests, run:

    ros2 launch wall_follower launch_test.launch.py


## Important in your wall_follower.py you need to add these 3 lines to your laser_callback function. This will allow the tests to update the desired side, distance, and velocity. These params should also not be manually overridden. 

    self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
    self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
    self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value


#### NOTE: 
There encrypted file is not being recognised by ros2. Just download the file: (your chip type does not matter)

`wall_follower_sim/wall_follower/np_encrypt.py` 

## Submission

There will be 6 log files created for 6 tests. These logs files will appear in your Ros2 workspace home: `racecar_docker/home/racecar_ws/TEST_NAME_log.npz` Submit all your test files to the [gradescope assignment](https://www.gradescope.com/courses/728544). 

(if you have not generated all the files because you have not passed all the tests you can still get partial points from submitting whatever files you do have)


