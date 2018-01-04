### 1. ROS CLI commands

*roscore*: starts master (parameter server)
*rosrun <node_name>*: launch a new node
   
*rosnode list*: lists all nodes available
*rosnode info <node>*: info about a node
*rostopic list*: lists all topics available
*rostopic info <topic>*: info about a topic (publishers, subscribers, ports they are running, message used, etc)
*rostopic echo <topic>*: prints topic messages to stdout
*rosmsg list*: lists all messages used
*rosmsg info <msg>*: info about a msg
*rosrun <node list>*: runs list of nodes (e.g., rosrun turtle_sim turtle_teleop_key)
*roslaunch <package_name> robot_spawn.launch*: runs package (needs to be executed from catkin_ws dir)
*rosdep check <package_name>*: checks runtime dependencies for a package (needs to be executed from catkin_ws	dir)
*rosdep install -i <package_name>*: installs all missing dependencies for a package (needs to be executed from catkin_ws dir)

### 2. Catkin workspace and packages

- Catkin_init_workspace: one-time command to build a workspace where to create [packages](http://www.ros.org/browse/list.php)
- The devel directory contains the setup.bash script,that must be sourced before using the catkin workspace
- Catwin workspace [directory structure](http://www.ros.org/reps/rep-0128.html)
- Catkin_make: to build packages (needs to be executed from catkin_ws dir)
- catkin_create_pkg <package_name>: to create packages (needs to be executed from catkin_ws/src dir)

ROS provides a large [list](http://www.ros.org/browse/list.php) of available packages

### 3. ROS Python library

[rospy](http://docs.ros.org/kinetic/api/rospy/html/)

### 4. Interacting with VM

ssh:
```
ssh student@localhost -p 2281
```
copy files:
```
scp -P 2281 ./arm_mover student@127.0.0.1:/home/student/catkin_ws/src/simple_arm/scripts/arm_mover
```

Note: openssh-server needs to be installed to be able to connect via ssh
```
sudo apt-get install openssh-server
```

### 5. Running the simulator

- Port forwarding needs to be configured in VirtualBox for VM, to forward all traffic destined to port 4567 in the host to 
port 4567 in the guest

- launch roscore, and then the package the enables communication between ROS in VM and simulator in host:

```
roscore
roslaunch launch/styx.launch
<launch simulator>
```


- Run packages as needed. E.g., waypoint_updater:
```
roslaunch waypoint_updater waypoint_updater launch
```

 