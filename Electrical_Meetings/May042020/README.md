# Meeting 

Members present: Elston, Mohit, Anand

Summary: Wifi connection to rover and options, LoRa, BluePill setup


## Introduction

Due to Covid 19 the university has limited student campus operations and so the build space in Hatch is unavailable. Since the Jetson TX2 is in hatch we are unable to access the hardware to make changes and test code. For this reason we are trying to tackle other issues such as the WiFi and LoRa modules.


## Wifi

The Jetson TX2 will be carrying a commercial antenna and the base station antenna will be team built as to minimize costs and maximize utility. The base station antenna requires to be have a hide rage of coverage in one specific direction. We have looked into a Yagi antenna on the the base station mounted on a lazy Susan attached to a small servo. We have considered implementations tackling the problem of having an uncontrolable axis, but for now we will consider the simplified case and build with what we have to get a foundation. If we assume that the Yagi antenna is able to work with a GPS sensor on the robot, then we are able to continuiously maintain a proper signal. Best case scenario is that if we are able to use GPS, Accelerometer, and  gyroscope data properly we may improve the accuracy of the anntenna direction.

https://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?referer=&httpsredir=1&article=1114&context=aerosp
https://arxiv.org/pdf/1704.06053.pdf
https://en.wikipedia.org/wiki/Kalman_filter


At the end of the week we will be considering different antennas for purchase


## LoRa

The LoRa modules have been delivered. We are hopefully going to have a proper breakout board by the end of the month and we are going to send the boards to members to get hands on experience


## Blue Pill

The Blue Pills have also arrived and we are going to uploading the bootloader so we can use Arduino libraries on them.










