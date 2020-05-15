# ROS Notes

* Assumes usage of the docker setup
* Pro-tip before starting: use a terminal with tabs!
* Installs with ROS using Python 2 (After error Python 3)
* Assumes installed "full-desktop"
* Unfortunately ROS keep throwing errors while getting rviz or rqt running. I tried debugging this for a long time, but have given up(for now) and decided to move to simply using the terminal applications instead. The error, gdb output, and my settings are provided below. This could possibly not be an issue for you so skip to the `rosrun` section to learn more!
* This file is created while following ROS Tutorials (Most of the tutorial is unfortunately stripped directly unless a section is particularly unclear)

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

~~You may also run the script in the landing directory after login~~

```
# ./ros_entrypoint.sh
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

* Packages: Packages are the software organisation unit of ROS Code. Packages contains libraries, executables, scripts, and other artifacts

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


To move into sub-directories
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

## Creating a ROS Package

ROS packages can either be made in catkin or rosbuild. For simplicity we are going to use catkin

* A package must contain a catkin compliant package.xml file
* A package must contain a CMakeLists.txt which uses catkin
* Each package must have it's own folder

Simplest folder structure:

```
my_package/
	CMakeLists.txt
	package.xml
```

### Packages in a catkin Workspace 

It is recommended to work with catkin packages using a catkin workspace. However, you may also build catkin packages as standalone

```
workspace_folder/
    src/
        CMakeLists.txt
        package_1/
            CMakeLists.txt
            package.xml
        ...
        package_n/
            CMakeLists.txt
            package.xml
```

In the source space (under the folder src), we have the top CMake file and all packages with their own CMake files

### Workspace Generation for catkin

To create a catkin workspace named catkin_ws, we simply make a file and run the `catkin_make` command

```
# cd /root/
# mkdir -p catkin_ws/src
# cd catkin_ws
# catkin_make
```

If we wanted Python 3 (not supported by default on the docker image), we may use  `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`.

### Creating a catkin Package

Ensure you are in the workspace created above in the source space (src folder)

```
# cd /root/catkin_ws/src
```

Use the following command to create a new package called `beginner_tutorials`
```
# catkin_create_package beginner_tutorials std_msgs rospy roscpp
```

This will create a folder with the package name along  with its package.xml and CMakeLists.txt partially filled  depending on the information provided to `catkin_create_pkg`

`catkin_create_pkg` is run with a package name followed by a list of optional dependencies. It will automatically ensure entries in the package.xml and CMakeLists.txt are created for the dependencies.

```
# catkin_create_pkg <package_name> [depend1] [depend2] ... [dependN]
```

### Building a catkin Workspace and Sourcing the Setup File

Going back to the root, we can run `catkin_make` to build the workspace

```
# cd /root/catkin_ws
# catkin_make
```

To add the workspace to the ROS environment 
```
# ./root/catkin_ws/devel/setup.bash
```
### Package Dependencies

To see **first order dependencies** we can use the `rospack` command
```
# rospack depends1 beginner_tutorials
```

We should expect to see the dependencies we listed during the creation
```
	roscpp
	rospy
	std_msgs
```

We can also view how adding the dependencies changes the package.xml

```
# roscd beginner_tutorials
# cat package.xml
```

Inside the package.xml file we may see the following tags:

* `<description> Package Description </description>`
* `<!-- Comments --> `
* `<maintainer email="email@email.email"> Name </maintainer>`
* `<license> Some license </license>`

The more important tags are:

* `depend`: Specifies a build, export, and exec dependency (most common)
* `build_depend` : Specify the other packages needed to build the current package
* `buildtool_depend`: Specify the tools the package needs to build itself  (usually catkin)
* `build_export_depend`: Specifies packages needed to build packages against this
* `exec_depend`: Specifies which packages are required at runtime in this package
* `test_depend`: Specifies optional dependencies for unit testing

To see all dependencies of a package we may run
```
# rospack depends beginner_tutorials
```

### Metapackages

Metapackages help to group multiple packages as a single local package. A metapackage has the following export tag in the package.xml

```
<export>
	<metapackage />
</export>
```

Other than `buildtool_depend`, metapackages only have `exec_depend`  packages that they group


The CMakeLists.txt file for a metapackage would look as follows
```
cmake_minimum_required(VERSION X.X.X)
project(<PACKAGE_NAME>)
find_package(catkin REQUIRED)
catkin_metapackage()
```

## Building a ROS Package

Ensure that we have properly setup our environment
```
# source /opt/ros/melodic/setup.bash
```

The `catkin_make` command can be used as follows
```
# catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

We may also optionally run the specific `install` target in the make file
```
# catkin_make install
```

If the `src/` folder is not in the local directory, then we may specify it 

```
# catkin_make --source <src directory>
```

### Building the Package

When all the dependencies of the packages are installed we run
```
# cd /root/catkin_ws
# catkin_make
```

## Understanding ROS Nodes

### Graph Concepts

* Nodes: A node is an exeutable that uses ROS to communicate with other nodes
* Messages: ROS data type used when subscribing or publishing a topic
* Topics: Nodes can *publish* to a topic as well as *subscribe* to a topic to recieve a message
* Master: Name service for ROS 
* rosout: ROS equivalent of stdout/stderr
* roscore: Master + rosout + paramter server

### Nodes

Nodes are simple executables that utilize the ROS client libary to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a service.

### Client Libraries

ROS client libraries allow nodes written in different programming languages to communicate:

* rospy  - Python client library
* roscpp - C++ client library

### ROS Core

`roscore` it the first thing one should run when using ROS (starts as the sole task in the initial docker container unless set to run in background)

### ROS Node 

As `roscore` runs in the background we continue with this section.

To list the nodes currently running we are able to use
```
# rosnode list
```

`roscore` provides the `/rosout` node and it always runs as to collect and log nodes' output

We can generate more information about a node by

```
# rosnode info /rosout
```
This provides what the node publishers, subscriptions, and services

Lastly, we can also see if the node is up or not by using `ping`

```
# rosnode ping rosout
```
The output is given below
```
xmlrpc reply from http://HOSTNAME:34959/	time=88.354588ms
xmlrpc reply from http://HOSTNAME:34959/	time=1.179457ms
...
```

### Ros Run

`rosrun` provides the ability to run a node directly from a package (without knowing the path to the package)

If we run 
```
# rosrun turtlesim turtlesim_node
```
we have now started running a new node meaning that it should be listed in the node list

```
# rosnode list
```
The output is given below
```
    /rosout
    /turtlesim
```

If we try to run a node that is already running, we will get an error
```
# roscore
```
```
RLException: roscore cannot run as another roscore/master is already running. 
Please kill other roscore/master processes before relaunching.
The ROS_MASTER_URI is http://HOSTNAME:11311/
The traceback for the exception was written to the log file
```

### Graphics Problems Encountered

At this point I had encountered a problem that I am not sure how to fix for my specific situation:

I was passing through my intel graphics for accelerated graphics and documentation of the steps below can be fond here:

http://wiki.ros.org/docker/Tutorials/Docker
http://wiki.ros.org/docker/Tutorials/GUI
http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
http://wiki.ros.org/action/fullsearch/rviz/Troubleshooting?action=fullsearch&context=180&value=linkto%3A%22rviz%2FTroubleshooting%22

For the following we need to ensure that the DISPLAY is forwarded to the docker container so we can run rviz and rqt.

Allow access for the docker container to access the screen:
```
# xhost +
```
Please remember to run the following command when you finished testing:
```
# xhost -
```
There is a much safer way to provide the display to the docker container, but this was for testing. 

The docker container command had to be modified to work with passing the display
```
# sudo docker run -it --rm --env "DISPLAY" --env="QT_X11_NO_MITSHM=1" --device=/dev/dri:/dev/dri --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name ros_main_test osrf/ros:melodic-desktop-full
```
Update and install mesa-utils and qt5
```
# sudo apt update
# sudo apt install mesa-utils qt5-default
```
I can run glxgears
```
# glxgears
```
which from the `rviz` troubleshooting page means that forwarding from the docker container is working correctly, but trying to run `rosrun rviz rviz` or `rqt` provides the following error
```
[ INFO] [1589416901.083238902]: rviz version 1.13.9
[ INFO] [1589416901.083274061]: compiled against Qt version 5.9.5
[ INFO] [1589416901.083283928]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1589416901.091102985]: Forcing OpenGl version 0.
dbus[1472]: The last reference on a connection was dropped without closing the connection. This is a bug in an application. See dbus_connection_unref() documentation for details.
Most likely, the application was supposed to call dbus_connection_close(), since this is a private connection.
  D-Bus not built with -rdynamic so unable to print a backtrace
zsh: abort (core dumped)  rosrun rviz rviz
```
By searching around and manually downloading all the tools, rebuilding ros in a different docker container, and rechecking all dependencies I was still unable to get `rviz` and `rqt` to work.

There could be an error since I was forwarding my intel graphics since my nvidia graphics are disabled for battery saving reasons, but the links provided above provide steps to follow for nvidia graphics as well.

```
Current setting:
	XPS 9570 (4K, intel i7, GTX 1050)
	OS: Ubuntu 19.04 (Disco)
```

Possible soltuions would be to look at:

https://hub.docker.com/r/moveit/moveit/tags for melodic
https://moveit.ros.org/install/docker/

https://github.com/pierrekilly/docker-ros-box

Or simply a software install which is what I am trying to avoid


The rest of the tutorial will cover nodes and their communication. Since this does not rely on rqt or rviz, this should be fine.

We will continue acting like this problem will be resolved. 

**The rest of this tutorial is being run on Arch Linux with ROS Melodic with Python 3**

##  Topics

Lets see an example to see how this can be used.

---------------------

If we run turtlesim in one terminal we get a turtle stuck at the center of the screen. 

```
# rosrun turtlesim turtlesim_node
```

We need a way for some other ROS node to tell the turtle how to drive. So we know that the turtlesim package has the `turtle_teleop_key` node

```
# rosrun turtlesim turtle_teleop_key
```
With this we now see the following text appear on the screen

```bash
	Reading from keyboard
	---------------------------
	Use arrow keys to move the turtle.
```
So as we keep the window running `turtle_teleop_key` running we are able to control the turtle on the screen via the arrow keys. Something to note is that if we change the focus to the `turtlesim_node` window and try to control it using arrow keys, it does not work. This means that inputs running from the terminal running `turtle_teleop_key` was being passed to the `turtlesim_node`.

--------------------

### ROS Data Flow 

The `turtlesim_node` and the `turtlesim_teleop_key` are both ROS nodes that are communicating over a ROS  **Topic** . The `turtlesim_teleop_key` is **publishing** key strokes and the `turtlesim_node` **subscribes** to the same topic to receive the key stroke data.

### rqt_graph

We can see this talk about publishing and subscribing by using `rqt_graph`



<img src="/home/ea/Dropbox/MarsRover/Docs/ROS/Images/rosgraph1.png" alt="rosgraph1" style="zoom: 67%;" />



Installing the package and running `rqt_graph` package 

```
# sudo apt install ros-melodic-rqt
# sudo apt install ros-melodic-rqt-common-plugins
```

We can simply run `rqt_graph`or we can start it through `rosrun`

```
# rqt_graph
```

```
# rosrun rqt_graph rqt_graph
```

By going this we get the graph shown above

In the graph above it shows that the `teleop_turtle` node is publishing to the topic `turtle1/cmd_vel` and the `turtlesim` node is listening for data being sent over the topic `turtle1/cmd_vel`

In this we can also change the options so as to plot the topic as it's own box and this is shown below

<img src="/home/ea/Dropbox/MarsRover/Docs/ROS/Images/rosgraph2.png" alt="rosgraph2" style="zoom:67%;" />

We can see that `turtle1` is a topic that contains data about the `cmd_vel` 

### rostopic

#### list

If we wanted to have a list of all the topics we can run 

``` 
# rostopic list
```

```
    /rosout
    /rosout_agg
    /statistics
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose
```

If we wanted more detail then we can run in verbose mode

```
# rostopic list -v
```

Verbose mode provides all the currently subscribed topics and the publishing topics

```
    Published topics:
     * /rosout_agg [rosgraph_msgs/Log] 1 publisher
     * /rosout [rosgraph_msgs/Log] 3 publishers
     * /turtle1/pose [turtlesim/Pose] 1 publisher
     * /turtle1/color_sensor [turtlesim/Color] 1 publisher
     * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher

    Subscribed topics:
     * /rosout [rosgraph_msgs/Log] 1 subscriber
     * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
     * /statistics [rosgraph_msgs/TopicStatistics] 1 subscriber
```

#### type

We know that messages are being sent from the node to the publisher, but what if we are interested in the type?

To get the type of messages being sent over a certain topic 

```
# rostopic type <topic>
```

In our specific case, if we wanted to see the data being sent over the topic `/turtle1/cmd_vel`

```
# rostopic type /turtle1/cmd_vel
```

We get the following

```
	geometry_msgs/Twist
```

Although not shown here, we can also see what data does this message type contain using `rosmsg show`. By doing this we are able to see that the type contains 2 arrays of float 64 numbers, 1 array of 3 float 64 numbers define the x, y, z of the linear movement and the other array of  3 floats define the x, y, z of the the angular movement.

#### echo

We can actually see the messages being sent using `echo`

```
# rostopic echo <topic>
```

In our case this would be

```
# rostopic echo /turtle1/cmd_vel
```

```
    ---
    linear: 
      x: 2.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
    ---
```



By doing this we can see that the messages from this topic is constantly being updated and shown whenever any new data is sent over the topic. This means that we have a node that is also listening to this topic now. If we refresh the rqt_graph then we are able to see that we have another node attached to the topic 

<img src="/home/ea/Dropbox/MarsRover/Docs/ROS/Images/rosgrap3.png" alt="rosgrap3" style="zoom:75%;" />



So we see that rostopic echo actually creates a node and subscribes to the topic to listen to the data

#### pub

We can use the following command to publish data to a topic 

```
# rostopic pub <frequency> <topic> <msg_type> <msg/args>
```

In our case we need to send one packet of data over the topic `/turtle1/cmd_vel`, the message type is `geometry_msgs/Twist` and the message would be `'[2, 0,0]' '[0,0,1.9]'`

```
# rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2,0,0]' '[0,0,1.9]'
```

The double dash ` -- ` is required to tell the option parser the rest of the text does not contain any options. This is required to ensure that the option parser does not take negative numbers as an option. If we knew that none of the messages contained negative numbers (not in quotes) then we could run it without the `--` and it would still work.

If we wanted to constantly send a message at a specific rate then we can use the `r` argument 

```
# rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2,0,0]' '[0,0,1.9]'
```

From now we see that this will constantly be sending data to the turtle every second. Since data is being sent through the topic we can see on the terminal window running `rostopic echo /turtle1/cmd_vel` the following is shown every second

```
    --- 
    linear: 
      x: 2.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 1.9
    ---
```

As is a constant in this section, we now look to see a new node publishing data to both the turtle and our `echo` node listening to the topic on the rqt_graph

<img src="/home/ea/Dropbox/MarsRover/Docs/ROS/Images/rosgraph4.png" alt="rosgraph4" style="zoom:75%;" /> 



Looking a bit more confusing, but if we show can also show the topic structure in rqt_graph to provide an additional level of abstraction

<img src="/home/ea/Dropbox/MarsRover/Docs/ROS/Images/rosgraph5.png" alt="rosgraph5" style="zoom:75%;" />

This looks much more cleaner!

#### hz

We can use the following command to determine the rate at which data is being published

```
# rostopic hz <topic> 
```

We are aware that `turtlesim_node` publishes data to `/turtle1/pose` let's find the rate at which it publishes the data

```
# rostopic hz /turtle/pose
```

```
    subscribed to [/turtle1/pose]
    average rate: 62.506
        min: 0.015s max: 0.017s std dev: 0.00044s window: 56
    average rate: 62.504
        min: 0.015s max: 0.017s std dev: 0.00047s window: 118
    average rate: 62.491
        min: 0.015s max: 0.017s std dev: 0.00046s window: 181
```



#### Piping rostopics

If we know the type of data being sent over a topic, it's not fun to type

```
# rostopic type /turtle1/cmd_vel
```

See the return value, then plug that value into `rosmsg show`

```
# rosmsg show geometry_msgs/Twist
```

We can instead pipe the output from one command to the other to get the same output as follows

```
# rostopics type /turtle1/cmd_vel | rosmsg show
```



### rqt_plot

We can graph the data from the topics we want by running `rqt_plot`

```
# rosrun rqt_plot rqt_plot
```

Then in the "Topic" bar type the topic you would like to plot the data form. If for example we wanted to plot the x and y data of the turtles position, we can plot the x and y cords from the topic `turtle1/pose`

<img src="/home/ea/Dropbox/MarsRover/Docs/ROS/Images/rqt_graph.png" alt="rqt_graph"  />

 The turtle is currently doing circles on the screen so if we zoom the x and y coordinates out, we should see sine graphs. We can also plot the angle theta which would look like a sawtooth graph as we reset back to 0 degrees every 2*pi

## ROS Services and Parameters





























