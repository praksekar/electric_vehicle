# Code for my Electric Vehicle Science Olympiad event. 

## How it works
This program relies on two adjustable parameters: 1) the target distance and 2) the desired acceleration rate. The reason why I only tweak these two parameters is because once they are set, there only one velocity vs. time graph the vehicle should emulate in order to run accurately and in the fastest possible time. At all times during the run, the vehicle should accelerate (and decelerate) at the highest possible rate before losing static friction with the ground. This acceleration parameter was eventually decided after testing at what point the accuracy becomes significantly poorer.

Based on these two parameters, a simple feedback algorithm is used to ensure the vehicle accelerates in the ideal manner I just explained above. First, the desired velocity vs. time graph that the vehicle will try to emulate during its run is generated. This function is shaped like an upside down symmetrical V, where the velocity increases linearly, peaks at a point, and then decreases linearly. The slope of this graph as its increasing or decreasing matches the provided acceleration rate parameter. During the run, the _actual_ velocity graph is also generated. In order to match the shape of the ideal graph, the motor power value is adjusted by some constant value times the difference between the ideal and actual functions at that instant. On top of this, there are several constraints programmed into this feedback loop to ensure the changes aren't too erratic and cause slipping. It is programmed to slow down to a crawl towards the last 20 cm of the run so it can stop more easily on the final target.

Was using a feedback loop approach necesssary? I'm not sure. If I had just set the motor power to some constant positive value, I think the vehicle would have acclerated uniformly, because a constant torque is being applied. Then, at the halfway point, I could have applied some constant negative value to the motor, causing a uniform deceleration. The effectiveness of approach could be verified by just graphing the velocity over time of the vehicle. If the graph is similar or better than the method I used, I would strongly recommend to just use this simpler method and avoid the headache of tweaking the feedback algorithm's parameters. 

## Awards and video
This vehicle won 1st place at the 2017 MIT invitationals and the 2017 Eastern Long Island Regional Final. It also got 2nd place at the 2017 UPenn invitational.

Here's a video of it performing at the 2017 Eastern Long Island Regional Final: https://www.youtube.com/watch?v=A2Euegb3qQg

## Construction
The overall construction of the chassis consisted of aluminum L angles and sheets. The mounts for the bearings, motor, and encoder are made from ABS 3D-printed plastic. The axles are made of threaded rod and are attached to robot wheels. An Arduino microcontroller with a motorshield was used to control the motor. A 5000mAh 7.2V LiPo battery powered the Arduino and the motor.
 
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


