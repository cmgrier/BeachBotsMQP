# BeachBots MQP
This is the repository for the BeachBots MQP
## Dependencies

## ROS
You can download any up to date versions of ROS for this project.
However, the Smallbots must use ROS Melodic on the Raspberry Pis.
You can find the instructions to downloading ROS Melodic [here](http://wiki.ros.org/melodic/Installation/Ubuntu) 

### RPi.GPIO
```
sudo apt-get update
```
```
sudo apt-get install rpi.gpio
```

### ZED Camera
If you want to just run the ZED samples, simply download the ZED SDK and follow the steps [here](https://www.stereolabs.com/docs/installation/) to run the samples.
If you want to be able to write your own code for the ZED and run it (which you will need to do for this project) you must have a NVIDIA GPU and will need to install CUDA 10.0 in addition to downloading the ZED SDK. The steps for doing so are [here](https://www.stereolabs.com/docs/installation/linux/).

### Computer Vision
https://coral.ai/docs/accelerator/get-started/#3-run-a-model-using-the-tensorflow-lite-api
## Code Architecture

### Basebot Package
This is where all the Basebot code is. The main node is the Director.py
file. This package includes the ZED camera nodes, task creation, zone creation,
robot communication,and anything that deals with mapping.

### Data Package
Any global data type used throughout the project can be found here. This
includes the Equal Priority Queue, Zones, and Task.

### Navigation Package
All navigation associated tools are stored in this package. The positional nodes,
imu.py and Encoder.py, are located here along with A* and Navigate. Navigate
uses all of these files to allow either the Basebot or Smallbot to drive a set
distance, turn to a specific angle, or drive to a coordinate.

### Small_bot Package
This package is as diverse as the abilities of the Smallbot. This package has
the SmallbotManager.py as the main node to run everything from the Smallbot side.
All the different types of task managers are here to execute all task types. 
The 2 degrees of freedom arm control is located here along with the Kinematics.py.
Finally all of the computer vision for trash detection is stored in the small_bot 
package.

### Support Package
This package has just anything that does not fit the other categories
and are useful. For instance the drive node is located here since it is
useable by both Basebot and Smallbot. However the highlight of this package
is the Constants.py file. This essential file holds all the constants of the
project and makes them global. This allows for easy changes across the project

### Test Package
The test package is where all the test files are. If a new feature is made,
the test for it should then be made here.

## How To Launch Basebot and Smallbot Code
To launch the Smallbot with all the code, ssh into the Smallbot and type:
```
roslaunch small_bot smallbot.launch
```
To launch the Basebot with all the code, ssh into the Basebot (or let your desktop
be the Basebot) and type:
```
roslaunch baseBot basebot.launch
```

## Wiki For More Code Details
[Beachbots MQP Wiki]()

