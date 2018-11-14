# ROS Beginner Tutorial - Publisher / Subscriber

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Overview

This project is basic implementation of a Subscriber and Publisher node in ROS. Also, It has a launch file node.launch and a service change_string which changes the publisher message.



# Dependencies

1. ROS Kinetic

2. System : Ubuntu 16.04

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

Don't forget to setup the ROS environment by adding the following line in your .bashrc :

```
$ source /opt/ros/kinetic/setup.bash

```

# Build and Run Instructions 

Following the environment setup, next step is to create the workspace.

```
$ cd <path where workspace needs to be created>
$ mkdir -p <name of workspace>
$ cd <workspace>
<workspace>$ catkin_make

```
Now, for building the main branch

```
<home>$ cd <workspace>/src
<workspace>/src$ git clone --recursive https://github.com/Indushekhar/beginner_tutorials
<workspace>/src$ cd ..
<workspace>$ catkin_make 

```

For building the Week11_HW branch, after cloning the main repo :

```
<workspace>/src$ cd beginner_tutorials
<workspace>/src/beginner_tutorials$ git checkout Week11_HW
<workspace>/src/beginner_tutorials$ ../..
<workspace>$ catkin_make

```



Just like the path of the ROS was sourced in .bashrc file, same needs to be done for the workspace by writing 

```
source <path to workspace>/devel/setup.bash
```
in the .bashrc. This will avoid the needs of sourcing everytime we run the package.

## Running using rosrun commands

Please ensure that rosmaster is running before executing following steps. rosmaster can be started by following command.

```
<home>$ roscore

```
To run the publsiher node, on an another terminal with a string argument which is to be published or else default prpgram message will be published. Only one of the string will be passed to the talker node.

```
<home>$ rosrun beginner_tutorials talker "Hello World"

```

To run the subsrciber node , on an another terminal 

```
<home>$ rosrun beginner_tutorials listener

```

## Running using roslaunch

To start both the nodes from single command, a launch file named node.launch has been created. Which can be executed as :

```
<home>$ roslaunch beginner_tutorials node.launch 

```

The launch file also except string as an argument, which will be published by the talker node. If the argument is not passed then the default argument value which is there in the launch file will be passed to the talker node. To run the node with launch file along with argument, run :

```
<home>$ roslaunch beginner_tutorials node.launch ss_msg:="String-argument-to-publish."

```
Please note that while passing the parameter to launch file the string should be a single cannot contain whitespaces.

## Calling the Ros Service

Service can be run from the terminal when the talker node is running. 

```
<home>$ rosservice call /change_string <string to be published>

```
For example, for publishing ``` "I just changed text from rosservice" ```, run :

```
<home>$ rosservice call /change_string "I just changed text from rosservice"

```

## Launching logger GUI

Logger GUI can be used to check the logged messages. To launch logger GUI, run the following lines:

```
<home>$ rosrun rqt_console rqt_console

```
## TF Transform

After running the talker node, we can check the transform tree and information published. Run the following command to check the transform information

```
<home>$ rosrun tf tf_echo /world /talk

```

To generate the tf tree, run 

```
<home>$ rosrun tf view_frames

```

## rostest/gtest

Test can be run in two ways.To run the tests, run following 

### 1. Using catkin_make

```
<home>$ cd <path to catkin workspace>
<workspace>$ catkin_make run_tests_beginner_tutorials

```
### 2. Using rostest

```
<home>$ rostest beginner_tutorials unit_test.launch

```
Output is shown below :

```
[Testcase: testunit_test] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-unit_test/serviceTest][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0
```

## Rosbag 

After successfuly build, You may run the command below to launch the nodes and record all the topics. The bag file will be in the results directory once the recording is complete. Run this launch file without any argument if you do not want to record the data. 

```
<home>$ roslaunch beginner_tutorials node.launch record:=true

```
Recording can be stopped anytime by typing CTRL+C in the terminal where above launch file was running.


One other way to record data is. First run the talker node and then run following on an another terminal :

```
<home>$ rosbag record -a --duration=15

```
You can specify the time ( in seconds ) for which you want to record the data.

We can check the information of the bag file using rosbag info <filename> command. Sample output looks like below:

```
indushekhar@indushekhar-GL502VMK:~/808X_ws/src/beginner_tutorials/results$ rosbag info talker.bag 
path:        talker.bag
version:     2.0
duration:    21.9s
start:       Nov 13 2018 16:26:49.71 (1542144409.71)
end:         Nov 13 2018 16:27:11.65 (1542144431.65)
size:        273.2 KB
messages:    1308
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      217 msgs    : std_msgs/String
             /rosout       439 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   435 msgs    : rosgraph_msgs/Log 
             /tf           217 msgs    : tf2_msgs/TFMessage

```


###  Playing back the bag file with the listener node running

Ensure that listener node is running. To see the data stored in the bag file recorded using the above command:

```
<home>$ cd catkin_ws/src/beginner_tutorials/results
<results>$ rosbag play talker.bag

```
This command will start playing the recorded bag file and the listener node which was running previously will be able to listen to the data that is being published by talker node.

