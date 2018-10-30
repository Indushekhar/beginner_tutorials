# ROS Beginner Tutorial - Publisher / Subscriber

# Overview

This project is basic implementation of a Subscriber and Publisher node in ROS.



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
Now, for building 

```
<home>$ cd <workspace>/src
<workspace>/src$ git clone https://github.com/Indushekhar/beginner_tutorials
<workspace>/src$ cd ..
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
To run the publsiher node , on an another terminal 

```
<home>$ rosrun beginner_tutorials talker

```

To run the subsrciber node , on an another terminal 

```
<home>$ rosrun beginner_tutorials listener

```
