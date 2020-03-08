

# CAN

### Modprobe to load the interfaces

`sudo modprobe can`

`sudo modprobe can_raw`

`sudo modprobe mttcan`

### Permanent Modprobe the modules

Edit the `/etc/modules-load.d/modules.conf` and add

`can`

`can_raw`

`mttcan`

### Setup Interfaces 

Setup the CAN interfaces

`sudo ip link set can0 type can bitrate 500000 dbitrate \`

`2000000 berr-reporting on fd on`

`sudo ip link set can1 type can bitrate 500000 dbitrate \`

`2000000 berr-reporting on fd on`

Enable the modified interfaces

`ip link set up can0`

`ip link set up can1`

There is a file in the home directory of the Jetson TX2 called `interfaceSetup.sh` which sets up the interface. Until we set a script to run on startup, for testing, we must call this script manually after startup.

`sudo sh ~/interfaceSetup.sh`



### Setup for Wiring

![schematic](https://i.stack.imgur.com/j74xh.png)



### Testing CAN

Hook up one transceiver to the CAN 0 interface pins on the Jetson TX2 to the CAN 1 interface pins via the CAN wiring diagram above. Always remember to have the 120 ohm resistors. 

The CAN pins are found on the J26 extension, not the J21 (labeled on the board).  

![Pinout Diagram](https://docs.fabo.io/jetson/JetPack3.2/TX2/setup/img/J26-2.png)

![https://i.imgur.com/nPUhhxz.png](https://i.imgur.com/nPUhhxz.png)



### Additional Links as Resources

https://developer.ridgerun.com/wiki/index.php/How_to_configure_and_use_CAN_bus

https://tekeye.uk/automotive/can-bus-loopback-test-with-pcan



