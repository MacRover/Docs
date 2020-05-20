# Server Explained

```c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
```
The `beginner_tutorials/AddTwoInts.h` is the header file generated from the srv file created earlier.

```c++
bool add(beginner_tutorials::AddTwoInts::Request &req,
	 beginner_tutorials::AddTwoInts::Response &res)
```

The function is the service for adding adding two ints. It takes the request and response defined in the srv file and returns a boolean.

```c++
res.sum = req.a + req.b;
return true
```

Two ints are added and the result is stored in the response

```
ros::ServiceServer service = n.advertiseService("add_two_ints", add);
```

The service is created and advertised over ROS.
