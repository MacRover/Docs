# ROS Docker Notes

* Installs with ROS using Python 2

* Assumes installed "full-desktop"


## Docker Startup

Run docker image

```
$ sudo docker run -it --name ros_main osrf/melodic-desktop-full roscore
```

Get into the container

```
$ sudo docker exec -it ros_main bash
```

Run the setup for the session

```
# source /opt/ros/melodic/setup.bash
```

## Catkin Workspace Generation

Create a new catkin workspace

```
# cd /root/
# mkdir -p catkin_ws/src
# cd catkin_ws
# catkin_make
```

## Installing ROS Packages

To install packages 

```
# apt-get install ros-melodic-<package-name>
```

Since we have "full-desktop" it comes pre-installed with the ros-tutorials package, but if you do not have the package

```
# apt-get install ros-melodic-ros-tutorials
```

Concepts:

* Packages: Packages are the software organization unit of ROS Code. Packages contains libraries, executables, scripts, and other artifacts

* Manifest (package.xml): A manifest is a description of a package. Defines dependencies between packages and to capture meta-information about the packages such as version, license, etc.


## ROS Navigation


To find package locations

```
# rospack find <package-name>
```

If we wanted to find the package `roscpp`
```
# rospack find roscpp
```
It would return `/opt/ros/melodic/share/roscpp`

In general, it would return 

```
YOUR_INSTALL_PATH/share/roscpp
```

The install path for ROS Melodic `/opt/ros/melodic`


In general, `rospack` is a powerful command to provide information about packages. To find a full detailed list of the capabilities of the command run:

```
# rospack help
```


To change directory to a package or stack

```
# roscd <location[/subdir]>
```

So if we wanted to change directory to the `roscpp` package

```
# roscd roscpp
```

We would be taken to the install directory `/opt/ros/melodic/share/roscpp`

ROS navigation tools will only find ROS packages that are within your `$ROS_PACKAGE_PATH`


To move into subdirectories
```
# roscd roscpp/cmake
```

You should be taken to `/opt/ros/melodic/share/roscpp/cmake`


To find the log files

```
# roscd log
```

Note: the log files will not exist until you run ROS programs


To `ls` ROS packages or directories

```
# rosls <location[/subdir]> 
```

If we wanted to see the files under the ROS package `roscpp-tutorials`

```
# rosls roscpp-tutorials
```

Tab completion is support and works as expected:

* Tab to complete the command
* For similar named commands, hit tab twice

To quickly see the packages installed

```
# rosls <tab>
```



