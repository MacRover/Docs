# Gazebo on ROS Melodic using ubuntu 18.04

#### This documentation is being compiled using online official resources using ROS <u>Melodic</u>. This will be regularly updated every 2-3 days with a focus on URDF and SDF along w/ ROS integration. 

<img src="https://www.ros.org/wp-content/uploads/2018/05/melodic_with_bg.png" alt="ROS.org | Install" style="zoom: 25%;" />![Gazebo](data:image/png;base64,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)

Using the new ROS noetic can be a little risky as it only a month old and is targeted towards Ubuntu 20.04. There is not a lot of documentation on it and you will run into many issues that can be a waste of your time.  Moreover not all of previous independent pre-built debian packages have been updated to run on noetic. ROS Melodic is the previous distro published, so it is fairly new with many packages and documentation and will give you the least trouble. 



However many books written on ROS have used the Kinetic distro as an example. As you expect, you might have trouble finding the correct debians to install for your project, a general way of getting the melodic version is replacing the distro name with the word melodic. For instance:

These are Pre-built debians targeted for the old ROS distro, Lunar:

```bash
$ sudo apt-get install ros-lunar-gazebo-ros-pkgs		
```

to get melodic version simply replace 'lunar' with 'YOUR_ROS_DSITRO':

```BASH
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs		

#### 
```

#### So what is Gazebo and why is it useful:

Gazebo is an independent software used by ROS. For each ROS distribution, there is a specific gazebo version that integrates well with the corresponding ROS distro.

From the open source robotics foundation (osrf) **"Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. While similar to game engines, Gazebo offers physics simulation at a much higher degree of fidelity, a suite of sensors, and interfaces for both users and programs."**

Gazebo also allows you to test and simulate your ROS package you have created for your actual robot. Moreover, Gazebo provides a set of ROS API's that allows users to modify and get information about various aspects of the simulated world. This is an amazing learning tool without having to worry about getting a physical robot. 

***Although you could, here is an interesting tutorial of 'hacking' a roomba and controlling it via ROS(Although it is quite outdated packages):***

##### Roomblock: a Platform for Learning ROS Navigation With Roomba, Raspberry Pi and RPLIDAR:

https://www.instructables.com/id/Roomblock-a-Platform-for-Learning-ROS-Navigation-W/

##### Someone also made their roomba a drone :

#### https://www.youtube.com/watch?v=wA2yIVFb2lI

You can also import your own robot that you have designed into the simulator and gazebo will let you add whatever sensors you want that are available 

<img src="https://res.infoq.com/articles/ros-2-gazebo-tutorial/en/resources/1ros-2-gazebo-tutorial001-1556868005585.jpg" alt="Open Source Robotics: Getting Started with Gazebo and ROS 2" style="zoom:25%;" />

#### Using a URDF in Gazebo:

The [Unified Robotic Description Format](http://www.ros.org/wiki/urdf) (URDF) is a useful and standardised  XML file format used in ROS to describe all elements of a robot. Unfortunately with robotics rapidly evolving, URDF format have not caught up and contains many shortcomings. You can read more about it here in the background section:http://gazebosim.org/tutorials?tut=ros_urdf.

It is important to understand URDFs to learn the newer XML format called the Simulation Description Format (SDF) was created for use in Gazebo to solve the limitations of URDF. Also URDF is heThe joint is defined in terms of a parent and a childavily documented in the ROS wiki but has not been updated to get around its limitations.

Creating a simple shape (URDF format):

```xml
   1 <?xml version="1.0"?>
   2 <robot name="myfirst">
   3   <link name="base_link">
   4     <visual>
   5       <geometry>
   6         <cylinder length="0.6" radius="0.2"/>
   7       </geometry>
   8     </visual>
   9   </link>
  10 </robot>
```

This is a robot called "myfirst" with only one 'link' (part) that is a 0.6 m long cylinder with a radius of 0.2 m.

You add multiple parts by adding more 'links':

```xml
   1 <?xml version="1.0"?>
   2 <robot name="multipleshapes">
   3   <link name="base_link">
   4     <visual>
   5       <geometry>
   6         <cylinder length="0.6" radius="0.2"/>
   7       </geometry>
   8     </visual>
   9   </link>
  10 
  11   <link name="right_leg">
  12     <visual>
  13       <geometry>
  14         <box size="0.6 0.1 0.2"/>
  15       </geometry>
  16     </visual>
  17   </link>
  18 
  19   <joint name="base_to_right_leg" type="fixed">
  20     <parent link="base_link"/>
  21     <child link="right_leg"/>
  22   </joint>
  23 
  24 </robot>
```

Here there are two links with their own specific geometry and one fixed joinet. The joint is defined in terms of a parent and a child.

By installing ROS melodic, gazebo 9 will be automatically installed. If you have the wrong version and want to uninstall gazebo use:

```bash
$ sudo apt-get remove gazebo11
```



Test if the standalone works:

```bash
$ gazebo
```

You should see the GUI open with an empty world. Check to see if you have the right version corresponding to your ROS dsitro:



Install the ROS-Gazebo integration packages: gazebo_pkgs(prebuilt debians)

ROS Melodic:

```bash
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

```

Documentation of using theese debians can be found here:https://github.com/ros-simulation/gazebo_ros_pkgs 

![img](https://github.com/osrf/gazebo_tutorials/raw/master/ros_overview/figs/775px-Gazebo_ros_api.png)



## Turtlebot3 (melodic): These instructions were tested on Ubuntu 18.04 and ROS Melodic (ISSUES MAY ARISE IF RUNNING WRONG GAZEBO VERSION )

##### https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

#### Creating a package that contains needed ROS dependencies:  

```bash
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```



***Useful tip: every time you open a new terminal, you have to source your package using: source ~/<u>package_name</u>/devel/setup.bash***

**You can let the terminal do this automatically by editing the .bashrc file:**

```bash
$ gedit ~/.bashrc
it will open a text file, scroll all the way to the bottom and add:
source ~/package_name/devel/setup.bash 

```

#### Simulate a TurtleBot3 World(Testing):

##### There are multiple TB3 robot models, 'burger', 'waffle','waffle_pi'

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

<img src="https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_waffle.png" alt="img" style="zoom:25%;" />

To control the model with keyboard: In a new terminal window (make sure you source your project):

```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```



#### Creating and building our own ROS package with a simple robot that uses sensors to detect objects and publishing a desired a message to a ROS node:



