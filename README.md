# ros_lawicel_canusb 
## Introduction  
This is a driver for Lawicel CANUSB in our Lab.   
The package have been tested on ROS-Kinetic on Ubuntu 16.04.  
About CANUSB's picture as follow:

![Lawicel](http://www.can232.com/wp-content/uploads/2013/01/Header_1.jpg "Lawicel")

## Principle 
- How to Send CAN message:   

Send messages to ROS Topic *"/can_tx"*  
- Read Receive CAN message:   

All the can message will be published on ROS Topic *"/can_rx"*  

## Pretreatment
If it is the first time to do the step, you should:
- cd into the launch file, for example:
```
cd ~/catkin/src/ros_lawicel_canusb/launch
```
- Execute the tty.sh file
```
./tty.sh
```
- Reset computer to make the rules file effective
```
sudo reboot
```

## Run  
- Register the can device
```
rosrun package_name canopen.sh
```
- Run our packed nodes
```
roslaunch package_name lawicel_canusb.launch
```

## Test  
- Install Can-utils   

You can find your way in http://elinux.org/Can-utils. 

- Test  

Help yourselves! If you have any questions please contact us or add issues. 


