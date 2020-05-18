# Talker Explained (`talker.cpp`)

Convenient headers for dealing with the ROS system
```
#include "ros/ros.h"
```
The following is includes the `std_msgs/String` from the `std_msgs` package. The headers are generated from the `String.msg` file in the package. 
```
#include "std_msgs/String.h"
```
The following is needed for properly dealing with strings using a string buffer
```
#include <sstream>
```
The `ros::init` function is required for initialization of the ROS system. We need to provide `argc`, `argv`, and a string. We need to provide `argc` and `argv` so that it may take command line arguments. The string provided is the node name that will be used. The name used cannot have a "/" in it. 
```
ros::init(argc, argv, "talker");
```
We need to create a handle to this node. The first `NodeHandle` created  will actually do the initialization of the node and the last one destructed will cleanup any resources the node was using. The `NodeHandle` is the main access point to communications with the ROS system. 
```
ros::NodeHandle n;
```
The `advertise()` function tells the ROS master that this node is publishing a message and so it updates the registry of publishers and subscribers. Any nodes currently subscribed to the topic this node is publishing to,  will start a p2p connection to this node. `advertise()` returns a `Publisher` object that allows the node to publish messages to a topic by calling `publish()`. Once all copies of the returned publisher are destroyed, the topic will automatically be unadvertised. 

In the code below, we tell the ROS master that we are publishing `std_msgs::String` messages to the topic `chatter`. We are allowing a queue of 1000 meaning that if we are trying to push messages faster than possible, it stacks in a queue up to 1000 messages. If the number of messages to publish overflows the queue, then we discard them.
```
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```
The `ros::Rate` object specifies at what rate to loop the messages being sent. In this case we are publishing messages at 10 messages per second. 
```
ros::Rate loop_rate(10);
```
Keep the count of the number of messages sent
```
int count = 0;
```
ROS has `ros::ok()` which handles SIGINT events that will become false when a SIGINT event is encountered

`ros::ok()` will returns false if:
* SIGINT event encountered
* Kicked off the network by another node of the same name
* `ros::shutdown()` has been called by another part of the application
* all `ros::NodeHandles` have been destroyed
```
while (ros::ok())
```
We first create a string message called `msg`
```
std_msgs::String msg;
```
Creates a stringstream object
```
std::stringstream ss;
```
Input the string "hello world" and the count into the `ss` variable. 
```
ss << "hello world " << count;
```
Set the string currently in the buffer of `ss` as the msg data.
```
msg.data = ss.str();
```
Display the message being sent. `ROS_INFO` is the replacement for `printf` or `cout`.
```
ROS_INFO("%s", msg.data.c_str());
```
We now use the `chatter_pub` variable that is a `Publisher` object to publish data into the topic which we defined above. The argument in `publish(arg)` is the message being sent to the topic.
```
chatter_pub.publish(msg);
```
We need to make sure that any callbacks are handled correctly. If the node was a subscriber then we must make sure it gets called properly.
```
ros::spinOnce();
```
Make sure to sleep for enough time to keep the 10Hz refresh rate
```
loop_rate.sleep();
```

Last Edited: May 17, 2020
Source: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
-- Elston Almeida