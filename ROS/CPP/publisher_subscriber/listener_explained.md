# Listener Explained (`listener.cpp`)

It is important to remember that the msg will always exist as long as references exist. `ConstPtr` is the type-def of `boost::shared_ptr<MSG const>`. We use this to avoid duplicating data so as to save memory and make sure that the object does not get destroyed as long as we have a reference to it.

```
std_msgs::String::ConstPtr &msg
```

In the library we also have `std_msgs::String::Ptr` which is `boost::shared_ptr<MSG>` 

```
void chatterCallback(const std::msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg -> data.cstr());
}
```

This function gets called whenever we have a node pushing data into the topic that this node is subscribed to. We take the constant pointer of the message and use `->` to access the data and format it as a `cstring`.

```
ros::init(argc, argv, "listener");
```

The `ros::init` function is required in every ROS node as it initializes a connection to the master(roscore). The function takes in command line arguments so we pass it `argc` and `argv`. Lastly, the name of the node is also declared as the last argument passed (string).

```
ros::NodeHandle n;
```

The `NodeHandle` object is the way we handle communications with the ROS system. The first node handle initializes the node and the deletion of the last node handle deletes the node.  

```
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```

`NodeHandle::subscribe()` returns a `ros::Subscriber` object that we keep until we decide to unsubscribe. We subscribe to the "chatter" topic and have a queue buffer of 1000 as when we cannot process the amount of data coming in, the data is stored in the queue. If there are more than 1000 messages waiting in the queue, then the extra messages are to be discarded. Each time a node publishes data on the topic this node is subscribed to, the `chatterCallback()` function is called. 

```
ros::spin();
```

Enters a loop checking for any messages this node is subscribed to. It does not use too much resources and will exit once `ros::ok()` returns false (conditions are listed in the `talker_explained.md`).



Last Edited: May 18, 2020

Source: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

-- Elston Almeida

