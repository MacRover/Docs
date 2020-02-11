# Meeting Notes (Feb 10, 2020)

Summary: Updates and new goals for the 2020 school year.


## Merger of Electrical and Software Communication Teams

The main reason why we decided to merge is due to the high amount of overlap in the work required. By merging the teams, we hope to increase productivity and collaboration removing the barrier of communication between the two teams. The software communication's team is still under the software branch, but will be operating on projects and working directly with the electrical communications team.


## Meetings

Biweekly meetings have been agreed to by all parties and weekly slack updates would be greatly appreciated. The meetings are going to be formatted in a style were the lead will allow others to take the stage and express their opinions so that we may discuss our ideas openly. We do not plan to stick to a hard plan and will change priorities based on the requirements of the team members.

## ROS

(Summary of ros wiki)

ROS (Robot Operating System) is robotics software platform that provides services expected from an operating system such as hardware abstraction, low-level device control, implementation of commonly-used functionality, inter-process  communication, and package management. One could think of ROS as a robot framework that provides all the tools required for managing data in a complex frameworks and includes tools such as synchronous data streams, data storage, etc.

ROS is not meant to be a framework with the most features, but rather to support code reuse in robotics research and development. Reasons to use ROS is that it is light, widely supported, language independent, builtin unit/integration testing, and scales easily. ROS runs on Unix-based platforms, but a port is available for Windows (Although don't)

#### Additional Readings
* [ROS Wiki Introduction](http://wiki.ros.org/ROS/Introduction)
* [ROS Wiki Tutorials](http://wiki.ros.org/ROS/Tutorials)
* [A Gentle Introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/)  

### General Knowledge and Applications

It has been showed off at multiple universities performing research on robotic arms, simulations, and robot surgery. Clearpath Robotics, a company located in Waterloo, has also integrated it with their Husky A200 robot.

### Our Application

Utilize ROS as a way to move data and make managing the robot easier once the foundations have been setup. With the focus on code reuse, we should be able to rewrite code for any part that requires to be used and simply add nodes for any additional functionality desired. In addition with the packages, simulation tools, and additional ROS compatible products should keep it relevant over time.

## LoRa

TLDR: LoRa is a wireless communication technology utilizing frequency modulation.

LoRa uses chirp spread spectrum modulation which encodes information by utilizing linear variations of frequency over time. Due to linear frequency variations, frequency offsets are equivalent to time offsets. LoRa is the lower physical layer that actually deals with the sending/receiving signals.

One does need to be aware that LoRa is designed for long distance low bandwidth communications so data rate is very low.

Factors for data transfer:

*Transmission Power* (power is inversely related)
*Data Rate(Bandwidth, Spreading Factor)* (bytes transmitted related)

Making the bandwidth 2x wider allows you to send 2x more bytes in the same time. Making the spreading factor 1 step lower allows you to send 2x more bytes at the same time. Lowering the spreading factor makes it more difficult for the reciever to receive a transmission, as it will be more sensitive to noise.

#### Additional Readings:
* [LoRa Gov Research Article](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5038744/)
* [LoRa Point To Point Research Article](https://arxiv.org/pdf/1909.08300.pdf)
* [LoRa Sender/Reciever Example using an ESP32](https://randomnerdtutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/)


### The Difference Between LoRa and LoRaWAN

LoRa is the physical communication techniques by sending methods using chrip spectrum modulation whereas LoRaWAN is the higher layers of communication that provide communication protocols, encryption, identification, etc. LoRaWAN allows for multiple LoRa nodes to be connected to a LoRaWAN gateway in a bidirectional communication setup.  

The LoRaWAN supports Bandwidths 125kHz, 250kHz, and 500kHz and has a network available in may cities that you could rent to send your own traffic through requiring no additional setup. The downside is that LoRaWAN is proprietary meaning that LoRaWAN gateways cost a lot and little/no support from the open source community.

### Applications and Additional Details

Created for military communications as it does not have a defined frequency peak meaning that they would be able to send messages covertly.

LoRa is being used for a lot of "smart city development" and IOT devices for communications.

City wide network in Calgary, IOT networks in Argentina, Brazil, Estonia, smart parking, smart busses, irrigation systems, space communications, people/sensor/asset tracking.


### Our Application

Since we are only utilizing the Rover and the base station, we may not require the extra features and cost incurred by choosing a LoRaWAN gateway, but rather just use point to point communications for sending low data rate traffic like position and other data which would not be time critical.


### Why do we need LoRa and Wifi?

LoRa module is mainly going to be sending simple data such as the position, encoder data, etc. Whereas the wifi module will be sending the video streams, operational data, errors, etc. We will leave the wifi module for only high bitrate transmissions and the LoRa module for time insensitive data.

## Documentation

Everyone should be pulling and pushing their code and contribute to the github repo as a way to keep track of progress and ensure that proper documentation/summaries and links are available for anyone to follow up on their work. Although not strictly required for all work to be documented, it would help to ensure that future team members and other teams can work along/keep updated/catch up in terms of progress. Documentation of research, libraries, and examples are going to help those who are interested in continuing the project as the years go on and we do not want them to start from scratch.

I expect everyone to post their changes or code they found(sourced) to folders on github. In our next session we may have a short introduction as to how to use github depending on the requirements.

Comments are important! If you need to write a little snippet to improve performance or if you would like to add some additional functionality, the best way would be to make the changes and add sufficiently detailed comments stating.


## Top Down View

   AFAIK ROS is the way we are able to control and integrate all the devices. We need to find a way for our ROS implementation to utilize CAN to communicate with other devices such as the motor encoders, LoRa modules, wifi-modules, and the battery management system. We should be able to get some LoRa communication working before the end of the month and hopefully get a good idea of ROS and it's capabilities before the end of the school year.

## Ideas

   The goal is to take the data off the robot, store in a database, and display it on a front-end so we can keep track of testing and find critical mistakes/improvements. This would be more of the end goal, but requires most of the ground work to be completed beforehand.

## Currently Work

Research specific parts relating to LoRa and ROS depending on topic found on the doc (not up yet)

