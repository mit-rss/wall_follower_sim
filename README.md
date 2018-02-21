# Lab 2: Wall Follower

Several common bugs have been resolved in [this piazza post](https://piazza.com/class/jdalf8rbr1o7jd?cid=142).

## Introduction

In this lab, you will be implementing a wall follower on a simulated version of the racecar. Your goal is to make an autonomous controller that drives the racecar forwards while maining a constant distance from a wall on either its left or right (chosen on the fly). It should also be robust to uneven surfaces and small errors in the LIDAR data, and it should be able to recover from small deviations from the desired state; being too far, too close, or too angled.

This lab is to be done individually. In the next lab, you will join your team to get your wall follower working on the real racecar.

We have made a series of tests to evaluate the performance of your wall follower. In order to test your code properly you must start out with the template starter code. The template is still very sparse and you have plenty of freedom to impliment any algorithm you'd like so long as the inputs and outputs are the same.

The lab is due Monday, 2/26 at noon.

## Update

We made small modifications to the racecar base software in making this lab. To get the updated code, navigate to the base directory and pull. Note that if you are using the VM, ```[YOUR_WORKSPACE]``` refers to ```~/racecar_ws/```, not the workspace you created in lab1c.
    
    cd [YOUR_WORKSPACE]/src/base
    git pull
    
Make sure you are on the "vm" branch in base by running ```git checkout vm``` in ```[YOUR_WORKSPACE]/src/base```.
    
## Download this Repository

Make sure you are in the ```[YOUR_WORKSPACE]/src``` directory. Then download this repository by clicking the "Download" link at the top of this page and unzipping the file. Set it up as your own git repository so you can push changes.

Your directory structure should now look like

- ```[YOUR_WORKSPACE]/src/```
  - ```CMakeLists.txt```
  - ```base/```
  - ```lab2_wall_follower_sim/```

Then rebuild your workspace by running ```catkin_make``` in ```[YOUR_WORKSPACE]```.

## Simulator

The simulator comes preinstalled with the racecar software. To run it, first launch the map server:

    roslaunch headless_simulator map_server.launch

Then begin the actual simulation:

    roslaunch headless_simulator simulate.launch 
    
If you get a ```KeyError``` when you try to run ```simulate.launch```, try to run ```teleop.launch``` first:

    roslaunch racecar teleop.launch
    
See the **Navigation** section for more on ```teleop```.
    
## RVIZ

To see the simulated car, you will need to open [```rviz```](http://wiki.ros.org/rviz). 
You can download the default settings for running the simulator in rviz [here](https://github.mit.edu/2018-RSS/racecar_base_ros_install/blob/vm/headless_racecar_sim/base.rviz). To download, hit "Raw" and then save the file as "base.rviz".

To open ```rviz``` with these settings, run:

    rviz base.rviz
    
These settings add place, drag and rotate tools so that you can manipulate the simulated car. If you save these settings to ```/home/YOUR_USERNAME/.rviz/default.rviz```, then you will have access to the tools simply by running ```rviz```. If all is working you should be able to see and manipulate the car like in the demo below (click to watch):

[![Simulator Demo](https://img.youtube.com/vi/A_xZnOwjMl4/0.jpg)](https://youtu.be/A_xZnOwjMl4)

You can toggle what is being displayed by checking and unchecking the boxes in the Displays panel. If you publish to a topic that can be visualized you can add it ```rviz``` by pressing the Add button at the bottom of the display panel.

![Add button](https://i.imgur.com/85tY4tZ.png)

## Navigation

You will need ```teleop``` running before the racecar can move on it's own. Launch it with 

    roslaunch racecar teleop.launch

With ```teleop``` running you can move the car manually with a usb joystick. For autonomous driving you can publish steering commands of type [```AckermannDriveStamped```](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) to the topic:

    '/vesc/ackermann_cmd_mux/input/navigation'
    
Import the ```AckermannDriveStamped``` type like this in python:

    from ackermann_msgs.msg import AckermannDriveStamped
    
## LIDAR

The racecar (and it’s simulation) are equipped a with LIDAR sensor that measures the distance from the racecar to its surroundings with high accuracy. All of the LIDAR data is published to the ```'/scan'``` topic.

The data is of type [```LaserScan```](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html). You can import the type in python using:

    from sensor_msgs.msg import LaserScan

The ```ranges``` data entry in the ```LaserScan``` message  is an array of the distances from the lidar sensor to the nearest obstacle. The measurements are taken at regular intervals, ```angle_increment```, from the angle ```angle_min``` to the angle ```angle_max```. The white points in this image are the laser scan as visualized in ```rviz```. Note that there is no data in the quadrant behind the car.

![LIDAR](https://i.imgur.com/k4jR51b.png)

## Steps to Success
How you implement the wall follower is entirely up to you. However implementing the following may get you started in the right direction:

* __Set up ROS structure__: Set up your wall follower node so that it subscribes to laser messages and publishes steering commands. Make sure you can at least make the racecar move fowards at a constant speed and turning angle before working on your controller.
* __Slice up the scan__: Consider slicing the ```ranges``` data into more useful pieces. A majority of the data won’t be useful to you if you only care about a wall to one side. When you can, try to use [```numpy```](https://docs.scipy.org/doc/numpy-dev/user/quickstart.html) operations rather than for loops in your code. [Multidimensional slicing](https://docs.scipy.org/doc/numpy-1.13.0/reference/arrays.indexing.html) and [broadcasting](https://docs.scipy.org/doc/numpy-1.13.0/user/basics.broadcasting.html) can make your code cleaner and much more efficient. You can turn any array into a ```numpy``` array with [```np.array```](https://docs.scipy.org/doc/numpy-1.13.0/reference/generated/numpy.array.html), or you can integrate it directly with ros like in [this tutorial](http://wiki.ros.org/rospy_tutorials/Tutorials/numpy).
* __Find the wall__: There are many ways to detect a wall in a laser scan. In a perfect world you might be able to detect it using a single sample of the LIDAR data. However with noisy data and uneven surfaces this might not be enough. A [least squares regression](https://en.wikipedia.org/wiki/Simple_linear_regression) is an easy way to account for more noise. The [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm can “upgrade” an existing model (like least squares) to be more robust to outliers. _Note: Attempt RANSAC only if you've already built a functional wall follower. It might be overkill_
* __Use a PD or PID__: A robust wall follower algorithm that can handle wavy wall contours and corners should probably use some sort of [PD or PID control](https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation). Simple P (proportional) control is often not enough to create a responsive and stable system. Think about the geometry of the wall following problem and how you could use it to get a better approximation of the D (derivative) term than if you used finite differences.

## Starter Code

Since we are providing you with a series of automated tests for your wall follower, we ask that you use the starter code contained in this repo and do not tweak it's original structure.

Make sure that your wall follower lives in the ros node initialized in this file:

    src/wall_follower.py

However if you want to add more python files to keep your code organized, feel free to do so.
You can import them into ```wall_follower.py``` just like any regular python files using ```import```.
    
You must use the following ros parameters in your follower:

* ```desired_distance```: The distance in meters the racecar should maintain from the wall
* ```velocity```: The speed the racecar should move in meters per second.
* ```side```: The side the wall is following represented as an integer. +1 represents the left wall and -1 represents the right wall. We chose this convention because typically we will assume the car is pointing in the positive _x_ direction. That means the left side of the car will point to the positive _y_ axis and the right side will point to the negative _y_ axis.  

We will test that your wall follower uses these constants correctly.
The default values of these parameters are stored in the ```params.yaml``` file.
Feel free to add more parameters if you choose to. You can access them in python using:

    MY_CONSTANT = rospy.get_param("/wall_follower/my_constant")
        
To run the wall follower launch:

    roslaunch wall_follower wall_follower.launch

## Running the Tests

The test suite we have made runs on entirely on it's own and puts your racecar through some basic wall following challenges.
Our test will override the values for ```desired_distance```, ```velocity```, and ```side``` that are set in the ```params.yaml``` file in order to make sure they are being used correctly.

First, kill all running ROS processes.
Then start ```roscore```. 
In another terminal run ```rviz``` if you wish to the see the racecar executing the tests.
Finally, run the following in a new terminal to begin testing:

    ./run_tests2
    
We have compiled 2 versions of the test binary (```run_tests``` and ```run_tests2```) for 2 different OS configurations.
The tests themselves are exactly the same.
```run_tests2``` should work on the VM and similar systems. If neither work please talk to the TAs.

Click on the video below to see a full run of the tests. 
Note that ```roscore``` and ```rviz``` are already running in the two stacked windows on the left.
In our implimentation we draw a red line in ```rviz``` using the [```Marker```](http://wiki.ros.org/rviz/DisplayTypes/Marker) class to visualize the wall that we have detected. You will be required to make useful visualizations like these in future labs. If you are having issues debugging your system it can be much easier to recognize the problem if you can visualize the output, so feel free to try to do so in this lab and ask the TAs if you need help.

[![Test Demo](https://img.youtube.com/vi/MkG3eMXFFsM/0.jpg)](https://youtu.be/MkG3eMXFFsM)

As the racecar completes each challege our script will write to a file named ```log.npz```. 
Upload this file to gradescope where it will be automatically graded.
If you kill the test script before you complete all of the challenges, the ones that have completed so far can still be graded.
If your racecar takes too long (i.e. it gets lost or crashes), the challenge will eventually time out and move to the next one.

To have a particular test case scored your racecar must successfully drive from the start point where we automatically place it, to a specified end area (see video for example).
In each test case we compute distances from the racecar to the wall on the appropriate side at regular time intervals.
If your code is truly following a wall than it should minimize the average absolute difference between the distance to the wall and the desired distance to the wall.

![Eqn1](https://latex.codecogs.com/gif.latex?loss=\frac{1}{N}\sum_{i=0}^N|distance[i]-desired\\_distance|)

To turn this value into a score between 0 and 1 we compute:

![Eqn2](https://latex.codecogs.com/gif.latex?score=\frac{1}{\alpha\cdot{loss}+1})

Don't worry, it is impossible to get exactly 100%.
In some test cases we start the racecar closer or farther to the wall than the desired distance, which will automatically lower the max score.
The racecar also has to navigate around tight turns which it can't do perfectly with a limited turning radius.
Moreover there are many ways to measure the distance to a wall so our metric might not match yours exactly.
We have chosen ![alpha](https://latex.codecogs.com/gif.latex?\alpha) so your score will be in the 90's if it is on par with the TA solution.
Your score for the ```short_left_far_angled``` test will be lower than the others because the racecar starts far from the desired distance.

Tampering with the autograder or the submission file and hardcoding solutions to the test cases will be considered cheating. Don't do it.
