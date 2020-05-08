# CAN

## Description

The Controller Area Network (CAN) was developed to prevent the need for large multi-core wiring harness in the automotive industry. 

The maximum CAN bus speed is 1 MBaud that can be achieved with a bus length up to 40 meters when using a twisted pair. The bus must be terminated at each end with a resistor of 120ohms. For bus lengths longer than 40 meters, the bus speed must be reduced. For our purposes in Mars Rover, we can easily run at 1MBaud as bus lengths are short. To put this into perspective, for a 1000 meter bus length you can use 50 KBaud bus speed. 

An example setup for a car is to use two different CAN busses. One for high speed where we would like to get as close to real-time operation as possible and one for low speed where speed is not as important. The high speed bus may be connected to the engine control, anti-lock breaks, transmission control, suspension, air bags, etc. While the low speed may take care of the power seats, locks, lighting, windows, etc. 



## Layers

Layers are a form of abstraction to introduce separation of concerns to complicated systems. In the Open Systems Interconnection (OSI) model used for computer networking there are 7 layers. From the top layer to the bottom we see: 

*   Layer 7: Application Layer
*   Layer 6: Presentation Layer
*   Layer 5: Session Layer
*   Layer 4: Transport Layer
*   Layer 3: Network Layer
*   Layer 2: Data Link Layer
*   Layer 1: Physical Layer

 An upper layer is said to be "on top" of the other layer. The lowest layer being Layer 1: Physical Layer. The Physical layer describes how the communication works on a fundamental level encompassing the electrical, mechanical, and the procedure interface. We will now shift to describe the Physical and Data Link layers for CAN communication.

Source:

Procedure Interface: https://www.ibm.com/support/knowledgecenter/en/ssw_ibm_i_71/rzasd/sc09250868.htm

Physical Layer: https://en.wikipedia.org/wiki/Physical_layer

OSI: https://en.wikipedia.org/wiki/OSI_model



## CAN Physical Layer 

The CAN protocol uses NRZ (Non-Return-to-Zero) bit coding. The signal is constant for one whole bit time and only one time segment is needed to represent one bit. We have two bus conductors "CAN_H" and "CAN_L" that have shifted waveforms driven differentially in balanced mode. 

* Logic 1 (recessive): No signal sent (logic 0 wins). Transceiver CAN_L floats to 2.5V and Transceiver CAN_H also floats to 2.5V. No voltage difference becomes realized as a logic 1.

* Logic 0 (dominant): Forces bus to zero level. Transceiver output at CAN_L driven to 1.5V. Transceiver output at CAN_H driven to 3.5V. Large voltage difference becomes realized as a logic 0.

Usually in NRZ, you may send "high" value for a logic 1 and a "low" value for a logic 0 as to follow intuition, but the line chip driver may invert these signals as to follow the CAN protocol. 

The main problem is that since CAN does not directly send a clock signal, if there was a long stream of zeros  or a long stream of ones, it does not have any reference as to how long a single bit may take. If there is a single bit that is sent independently, then the receiver is able to understand the bit rate using the boundaries of the bit and make sense of the message being sent. We will see how bit stuffing is used to resolve this problem.

Bit stuffing is when the transmitter automatically adds a bit of the opposite polarity when a sequence of 5 bits are sent with the same polarity. So this means that the message 0111111 becomes 01111101. We can see the second sequence contains an extra bit being a zero. This provides the boundaries the receiver needs to correctly decode the message.  The receiver automatically removes these additional bits to read the true message being sent. Bit stuffing relies on the fact that no error occurs in the wire transmission as an inverted bit may corrupt the entire serial communication. This means that electromagnetic interference should be carefully dealt with to ensure reliability. A cycle redundancy check is usually used to minimize any errors. 



## CAN Data Link Layer

A message begins at the micro-controller sending a stream of bits to the TX line (Transmit Line) to the transceiver. The transceiver converts the bits to voltages and amplifies the signal so that the bits can be sent over the bus serially. The transceiver on the receiving end converts the voltage values sent to a bit stream and sends it to the micro-controller over the RX line (Receive Line) 

It is important to note that a message sent by the broadcaster can be received by any micro-controller (or control unit) attached to the CAN_H and CAN_L busses. 

### CAN Frame Format 

The CAN Frame Format is structured as follows:

*   Start Bit (1b): == 0

*   Message ID (11b for Jetson TX2)

*   Control fields (3b)
*   Data Length (4b)
*   Data (0-64b ? 32b)
*   CRC (Circular Redundancy Check) (15b)
*   ACK (acknowledgement bits) fields (3b)
*   End of Frame Delimiter (7b)

All message IDs are unique (centrally assigned), lowest message ID has the highest priority. Any devices connected on the CAN bus can filter based on message ID and discard any frames with frame or CRC errors.

### CAN MAC (Medium Access Control)

This section describes how CAN works communicating data between devices.

CAN is based on a half-duplex distributed architecture. 

Devices on the bus are not specified with an address, but rather the address information is contained in the identifiers of the messages that are being transmitted. Thus the reason why each message must contain a message ID that describes the message and the priority. Lack of addresses allows for changes to the devices on the CAN bus without disturbing communication within existing devices. 

When a device does not have any data to send ( or between frames), it sends at least 3 recessive bits (logic 1). This time allows devices to perform processes before the next message.

CSMA/CD - Carrier Sense Multiple Access / Collision Detection

The CAN communication protocol is a CSMA/CD protocol meaning that every device on the network must monitor the bus for a period of no activity before trying to send a message on the bus (Carrier Sense). Onces the period of no activity ends, each device on the bus has an equal opportunity to transmit a message (Multiple Access). If two nodes on the network start transmitting at the same time then it will be detected and corrected (Collision Detection). When a collision occurs non-destructive bitwise arbitration occurs to ensure messages remain in action even if a collision occurs. 

Since we already discussed 0 is the dominant bit, it will be preferred in arbitration over a recessive bit. This means that the messages with the lower message ID (higher priority) always gets sent first. All devices monitor the bus when trying to send a message. This means that if a device is trying to send a lower priority message (recessive bit) onto the bus and notices that a dominant bit exists on the bus it immediately stops transmitting and waits for the next period of no activity. 

CAN specifies 4 types of frames:

*   Data - Broadcasts data to bus
*   Remote - Request data from device 
*   Error - Reports an error by a device
*   Overload - Generally Not Used

CAN Specifications:

*   CAN version 2.0A specified an 11 bit message identifier
*   CAN version 2.0B suports both 11 and 29 bit message identifiers
*   CAN High Speed ISO 11898, for speeds between 125 kbps and 1Mbps
*   CAN Low Speed ISO 11519-2, for speeds up to 125 kbps

Summary from Sources:

CAN: https://erg.abdn.ac.uk/users/gorry/eg3576/CAN-phy.html

Balanced and Differential Transmission: https://erg.abdn.ac.uk/users/gorry/eg3576/balanced.html

Cycle Redundancy Check: https://erg.abdn.ac.uk/users/gorry/eg3576/crc.html

CSMA/CD protocol: https://www.sciencedirect.com/topics/earth-and-planetary-sciences/carrier-sense-multiple-access



## CAN Jetson TX2 Summary

Standard: ISO 11898-1:2006/11898-1:2015 Road vehicles -- Controlling Area Network (CAN)

CAN controller comes pre-tested at 1-Mbps CAN PHY

Features:

* CAN protocol version 2.0A, version 2.0B and ISO 11898-1:2006/11898-1:2015

  -   Dual Clock Source FM-PLL designs

  -   16, 32, 64, 128 Message Objects (configurable)

  -   Each message has it's own identifier mask

  -   Programmable FIFO mode

  -   Programmable loop-back mode for self-test

* Parity check for Message RAM (optional)

  -   Maskable Interrupt
  -   MA support, automatic Message Object increment (not sure what is MA)
  -   Power-Down support

* Supports TT CAN

  -   TTCAN Level 0,1,2

  -   Time Mark Interrupts

  -   Stop Watch

  -   Watchdog Timer

  -   Synchronization to external events

-------------------------------------
| Signal Name | type  | Description |
|-------------|-------|-------------|
| CAN WAKE    | Input | Wake        |
| CAN RX [0:1]| Input | CAN RX Bus  |
| CAN TX [0:1]| Output| CAN TX Bus  |
| CAN ERR     | Input | Error       |
| CAN STBY    | Output| Standby     |
-------------------------------------



Source: https://developer.download.nvidia.com/assets/embedded/secure/jetson/TX2/docs/NVIDIA_JetsonTX2_Developer_Kit_Carrier_Board_Specification.pdf?1WFskgFABPHichkjGzoM47tVH2jaEfxROrVek02d635ftnc3OXUbikM272f0SVnT7mf5rhcGZuR84zB8IBuViGPdX-IPIXsZ6XE8HSyfQRdrW95kWMQ5G_w7_qD0a-LLlJgAzrGDpxoYQcvsY6HxeD2r5sKDfh8FTd59ebepaWmrliq7a0vXHd7jL38Wk-XMw_m-RFhK8BgkUHyfTh6aQ_b1sIs

## Setup and Testing

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



### Wiring

<img src="https://i.stack.imgur.com/j74xh.png" alt="schematic" style="zoom:75%;" />



### Testing CAN

Hook up one transceiver to the CAN 0 interface pins on the Jetson TX2 to the CAN 1 interface pins via the CAN wiring diagram above. Always remember to have the 120 ohm resistors. 

The CAN pins are found on the J26 extension, not the J21 (labeled on the board).  

<img src="https://docs.fabo.io/jetson/JetPack3.2/TX2/setup/img/J26-2.png" alt="Pinout Diagram" style="zoom:18%;" />

![https://i.imgur.com/nPUhhxz.png](https://i.imgur.com/nPUhhxz.png)



### Additional Links as Resources

https://developer.ridgerun.com/wiki/index.php/How_to_configure_and_use_CAN_bus

https://tekeye.uk/automotive/can-bus-loopback-test-with-pcan



##  CAN Bus communication Technical Resources 

https://drive.google.com/drive/folders/1te6AMZTjTfA6JJ2Zapu5d2f9G_nPo6H3?usp=sharing

https://canopen.readthedocs.io/en/latest/network.html

## Implementing CAN Bus communication with Victor SPX Motor Controllers: 

https://phoenix-documentation.readthedocs.io/en/latest/ch08_BringUpCAN.html

To interact with devices on the CAN bus using python use this library:

https://github.com/normaldotcom/CANard/

One should consider if it is worth rewriting the library specific for our purposes or writing a wrapper.

## CanOpen Implementation with ROS:

https://www-csr.bessy.de/control/Hard/fieldbus/CAN/CiA/Bessy/DSP402.pdf

https://github.com/ipa-mdl/ros_canopen

http://wiki.ros.org/ros_canopen

One thing we'll probably have to look into in the future is SocketCAN and the ros_canopen package.