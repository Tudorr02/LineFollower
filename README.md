# LineFollower
> Robotics project 


As part of an introduction to robotics project, we developed a line follower robot using an Arduino Uno. This was a great way to learn and apply basic robotics concepts.

The robot is built on a simple cardboard structure, thus providing a light and accessible base for the electronic components. We used a mini breadboard to connect the components, jumper wires to facilitate electrical connections, and a ball caster to provide stability to the robot.

## Project Components
- Arduino Uno: The heart of the project, used as a central control unit.
- Mini Breadboard: To facilitate electrical connections.
- Jumper Wires: Essential for connecting components.
- Ball Caster: For mobility and balance.
- Two Motors with L293D Driver: Provide the robot's movement.
- LiPo Battery: Power source.
- QTR Sensors: Vital for line detection.

## Design and Construction
We built the robot on a cardboard base, creating a lightweight and adaptable platform. We integrated the electronic components on the mini breadboard, and the motors were mounted on the sides for propulsion. The ball caster added the necessary stability for the robot's movements.

## Software Part
We developed a software part in which we included the QTRSensors library and defined the pins for motors and sensors. Within the PID (Proportional-Integral-Derivative) system, which is essential for controlling the robot's movement, we added an additional function for error handling. This function was specially designed to cope with situations where the robot encounters curves on the route.

By fine-tuning the PID values - kp, ki, and kd - we managed to ensure a smoother and more precise line tracking. However, to make the robot more adaptable to sudden changes in direction, such as curves, we implemented an algorithm that dynamically adjusts the motor speed based on the magnitude and frequency of detected errors. This means that when the robot detects a curve, it adjusts the motor speed to negotiate it more efficiently, thus avoiding deviating from the route.

Auto-calibration, which we integrated into the robot, also contributes to this adaptability, allowing the robot to recognize different types of surfaces and lighting conditions. This is crucial in situations where the followed line passes from one surface to another or is affected by changes in light.

## Challenges and Learnings
The main obstacle was the fine-tuning of the PID parameters - kp, ki, and kd - to ensure a smooth and precise line tracking. Auto-calibration added a level of complexity, but at the same time, it increased the efficiency and robustness of the robot. By overcoming these challenges, we gained valuable knowledge in the field of robotics and developed problem-solving and teamwork skills.

## Photos and Videos
 

What it looked like before we hit all the walls:

<img src="https://github.com/Tudorr02/LineFollower/assets/92024989/211d5211-9556-4180-af7e-02b8d3272f3a" width="500" height="500">

 
What it looked like after:

<img src="https://github.com/Tudorr02/LineFollower/assets/92024989/45f22666-2a15-4371-86ac-2ea13a836fb8" width="500" height="500">
<img src="https://github.com/Tudorr02/LineFollower/assets/92024989/35c1c7b6-bc29-4cff-8dff-01e55d2b5139" width="500" height="500">

### **The best time achieved on the dinosaur-shaped circuit : 21.381**

Demonstration : https://youtube.com/shorts/5sdWQ27HN_0?feature=share

