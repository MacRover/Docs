# Publisher & Subscriber Example (C++)

In this folder you can find the example of a publisher and subscriber written in C++. 

A more detailed explanation of the code for `listener.cpp`  is provided in `listener_explained.md`

A more detailed explanation of the code for `talker.cpp` is provided in `talker_explained.md`

Both of these files are to be under `beginner_tutorials/src` and we edit the CMakeLists.txt file of the beginner_tutorials to include at the bottom:

```
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

Complete CMakeLists.txt:

```
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

The two executables for talker and listener will go under the devel space. 

We need to make sure the libraries we need to build the executable are linked

```
target_link_libraries([talker/listener] ${catkin_LIBRARIES})
```

Since we are using messages from other packages inside the catkin workspace, we need to make sure that the message headers are generated before begin used. Since catkin builds in parallel, we need to make sure to report dependencies to avoid compile errors. 

```
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

If you notice, there is no such package named `beginner_tutorials_generate_messages_cpp` , but we call it as a dependency. We should note this is a CMAKE target and not a complete package. If we go up in the CMakeLists.txt file we see

```
generate_messages(DEPENDENCIES std_msgs)
```

so we invoke this to generate messages for C++.



Last Edited: May 17, 2020

Source: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

-- Elston Almeida

