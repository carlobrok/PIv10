# PIv10
Computer vision algorithm for RoboCup Rescue Line.

## Requirements

* Raspberry Pi
* Raspbian or other Linux operating system
* OpenCV 4 or newer Version (recommended), [Install guide for OpenCV 4.1.0](https://docs.opencv.org/4.1.0/d7/d9f/tutorial_linux_install.html)  
  **Important:** after finishing the installation run `sudo ldconfig`
* WiringPi, install with `sudo apt-get install wiringpi`
* Boost, install with `sudo apt-get install libboost-all-dev`

## VideoServer 

In the VideoServer class, a new thread is started, so the output images can be streamed from the Raspberry Pi's IP-address using a VideoClient program (not published).


## Issues 

The green point can be detected pretty sure, but when the robot is in the wrong angle to the intersection, a diagonally opposite green point is interpreted as a side by side double green point.  
Besides, the 5 ROIs (region of interest) are not the perfect solution.  
  
That is why I stopped developing these algorithms and started a new approach to detect a double green point and track the line.  
The new algorithm will be published in mid 2020, after the RoboCup 2020.

## Code explanation / code breakdown

**Initialize variables, VideoServer and CameraCapture threads**

```cpp
#include <iostream>     // std::cout, etc.
#include "opencv2/opencv.hpp"   // opencv library
#include "boost/asio.hpp"     // used for the VideoServer
#include <thread>     // multithreading
#include <chrono>     // timing etc.
#include "VideoServer.h"    // VideoServer for image streaming
#include "CameraCapture.h"    // captures the camera image in the background
#include "wiringPi.h"       // wiringPi library for communication with Arduino Mega (interrupt pin)
#include "wiringSerial.h"   // wiringPi library for serial communication
```
1. Inlcude all header files for communicaton and image processing  
</br>

```cpp
CameraCapture cam(0);
cam.set(cv::CAP_PROP_FPS, 30);
cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
```

2. Open the [Raspberry Pi camera](https://www.raspberrypi.org/products/camera-module-v2/). Set frame rate to 30fps and image size to 640 by 480 pixels.  
</br>

```cpp
int fd = serialOpen("/dev/serial0", 115200);

if(fd < 0){
  std::cout << "Fehler beim Öffnen der seriellen Schnittstelle" << std::endl;
  return 1;
}

if (wiringPiSetup() == -1) {
  std::cout << "Fehler beim wiringPi Setup!" << std::endl;
  return 1;
}

pinMode(5, OUTPUT);
digitalWrite(5, 0);
```
3. Open serial device "serial0" wich is the GPIO serial port of the Raspberry Pi with 115200 baud rate.
4. Set GPIO pin 5 as OUTPUT and write the pin low.
</br>

```cpp
Mat img_rgb;
Mat img_bin;
Mat img_hsv;
Mat img_sw;
Mat img_binsw;
Mat img_gminb;		 
Mat img_gminr;		 
Mat img_unterschied; 
Mat img_binunt; 
Mat img_binhell;
```
5. Create opencv *Mats* in wich filtered and converted images will be saved.
</br>

```cpp
Rect rois[5];

std::vector< std::vector<Point> > contg;
std::vector< std::vector<Point> > cont1;
std::vector< std::vector<Point> > cont2;
std::vector< std::vector<Point> > cont3;
std::vector< std::vector<Point> > cont4;
std::vector< std::vector<Point> > cont5;
```
6. Create ROIs, region of interests and vectors to hold black and green contours.
</br>

```cpp
Scalar lowGreen = Scalar(50,90, 45);
Scalar upperGreen = Scalar(90, 255, 110);
```
7. Scalar in wich HSV color range the color green is. Later used for opencv function *inRange*
</br>

```cpp
VideoServer srv;

srv.namedWindow("RGB");
srv.namedWindow("Bin");
srv.namedWindow("SW");
srv.namedWindow("BinSW");
srv.namedWindow("HSV");
srv.namedWindow("Hell");

while(!cam.read(img_rgb));
```
8. Create VideoServer, add Windows and wait for the first camera image.
</br>

---

**Main processing loop / while(1) loop**


```cpp
while(!cam.read(img_rgb)) {}

if(serialDataAvail(fd) > 0){
  char c = serialGetchar(fd);

  if(c != '\n' && c!= '\r') {
    std::cout << c << "   ";
    if(c == 'A') {
      serialPutchar(fd, 'A');
      std::cout << "Beginning of communication." << std::endl;
      digitalWrite(5, 0);
      serialPutchar(fd, 'B');
    }
    else if(c == '0') {
      digitalWrite(5, 0);
      std::cout << "pin 5 Low" << std::endl;
      senddata = true;
    }
  } else {
    std::cout << std::endl;
  }
}
```
1. Wait for next image
2. Read serial input buffer and set senddata to true, when recieved a '0' to perform image processing.
</br>
