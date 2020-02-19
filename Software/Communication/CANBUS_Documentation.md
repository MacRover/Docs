<h1>Implementing CAN Bus communication with Victor SPX Motor Controllers: </h1>
https://phoenix-documentation.readthedocs.io/en/latest/ch08_BringUpCAN.html

To interact with devices on the CAN bus using python use this library: 
https://github.com/normaldotcom/CANard/

One should consider if it is worth rewriting the library specific for our purposes or writing a wrapper.

#CanOpen Implementation with ROS:

https://www-csr.bessy.de/control/Hard/fieldbus/CAN/CiA/Bessy/DSP402.pdf

https://github.com/ipa-mdl/ros_canopen

http://wiki.ros.org/ros_canopen

One thing we'll probably have to look into in the future is SocketCAN and the ros_canopen package. 
