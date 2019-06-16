# PIv10
Computer vision algorithm for RoboCup Rescue Line.

## Requirements

* Raspberry Pi
* Raspbian or other Linux operating system
* OpenCV 4 or newer Version (recommended), [Install guide for OpenCV 4.1.0](https://docs.opencv.org/4.1.0/d7/d9f/tutorial_linux_install.html)
* WiringPi, install with `sudo apt-get install wiringpi`
* Boost, install with `sudo apt-get install libboost-all-dev`

## VideoServer 

In the VideoServer class, a new thread is started, so the output images can be streamed from the Raspberry Pi's IP-address.


## Issues 

The green point can be detected pretty sure, but when the robot is in the wrong angle to the intersection, a diagonally opposite green point is interpreted as a side by side double green point.  
Besides, the 5 ROIs (region of interest) are not the perfect solution.  
  
That is why I stopped developing these algorithms and started a new approach to detect a double green point and track the line.  
The new algorithm will be published in mid 2020, after the RoboCup 2020.

