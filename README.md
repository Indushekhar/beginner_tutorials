# ROS Beginner Tutorial - Publisher / Subcriber

# Overview

This project is basic implementation of a Subsciber and Publsiher node in ROS.



# Dependencies

1. ROS Kinetic

2. System : Ubuntu 16.04

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

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
