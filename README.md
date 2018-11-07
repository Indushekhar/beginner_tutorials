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

# Running using roslaunch

To start both the nodes from single command, a launch file named node.launch has been created. Which can be executed as :

```
<home>$ roslaunch beginner_tutorials node.launch 

```

The launch file also except string as an argument, which will be published by the talker node. If the argument is not passed then the default argument value which is there in the launch file will be passed to the talker node. To run the node with launch file along with argument, run :

```
<home>$ roslaunch beginner_tutorials node.launch ss_msg:="String argument to publish."

```

# Calling the Ros Service

Service can be run from the terminal when the talker node is running. Only one string without spaces will be passed to code.

```
<home>$ rosservice call /change_string <string to be published>

```


# Launching logger GUI

Logger GUI can be used to check the logged messages. To launch logger GUI, run the following lines:

```
<home>$ rosrun rqt_console rqt_console

```

