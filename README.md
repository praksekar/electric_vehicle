# Code for my Electric Vehicle Science Olympiad event. 

## How it works
The code is pretty long, I know, but the general idea is that we tweak certain parameters, whether its the max coefficient of friction that I believe the wheels can handle with accuracy or the maximum time I want to achieve. Based on this, it generates a desired velocity versus time function for the vehicle's run. It also uses the basic physics kinematics formulas (vf^2 = vi^2 + 2ad and T = Iα), and solves for the acceleration as a function of the motor power setting. Then, it checks whether the graph current vehicle's velocity (based on encoder data) matches the desired velocity curve. If not, it tweaks the motor power value by some calculated factor times the difference between the current and ideal functions to adjust the acceleration. On top of this, there are several constraints programmed into this feedback loop to ensure the changes aren't too erratic and cause slipping. It is programmed to slow down to a crawl towards the last 20 cm of the run so it can stop more easily on the final target.

## Awards and video
This vehicle won 1st place at the 2017 MIT invitationals and the 2017 Eastern Long Island Regional Final. It also got 2nd place at the 2017 UPenn invitational.

Here's a video of it performing at the 2017 Eastern Long Island Regional Final: https://www.youtube.com/watch?v=A2Euegb3qQg

## Construction
The overall construction of the chassis consisted of aluminum L angles and sheets. The mounts for the bearings, motor, and encoder are made from ABS 3D-printed plastic. The axles are made of threaded rod and are attached to robot wheels. An Arduino microcontroller with a motorshield was used to control the motor. A 5000mAh LiPo battery powered the Arduino and the motor.
 
Overall views:
![image](https://user-images.githubusercontent.com/21281736/110042209-6f4b7980-7d13-11eb-8534-7a3b565ef5d3.png)
![image](https://user-images.githubusercontent.com/21281736/110042428-c3eef480-7d13-11eb-9d95-84d6f392ac27.png)

Caliper to adjust steering angle: 
![image](https://user-images.githubusercontent.com/21281736/110042610-07e1f980-7d14-11eb-91ad-a06927847da3.png)

Rotary encoder to get current position and speed: 
![image](https://user-images.githubusercontent.com/21281736/110042260-85f1d080-7d13-11eb-9134-485089ba8986.png)

Laser pointer for aiming:
![image](https://user-images.githubusercontent.com/21281736/110042300-9609b000-7d13-11eb-83e9-5ae4a138f9e2.png)

Motor:
![image](https://user-images.githubusercontent.com/21281736/110042367-ae79ca80-7d13-11eb-8dca-a91163d40052.png)

Motor Shield for Arduino: 
![image](https://user-images.githubusercontent.com/21281736/110042317-9dc95480-7d13-11eb-8cd4-e08c8727b2c0.png)


