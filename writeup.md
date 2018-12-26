## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/arm_pick.png
[image5]: ./misc_images/arm_drop.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Properties extracted from kr210.urdf.xacro
joint | parent link | child link | rotation axis | frame origin (x - y - z)
--- | --- | --- | --- | ---
1 | base | 1 | z | 0 - 0 - 0.33
2 | 1 | 2 | y | 0.35 - 0 - 0.42
3 | 2 | 3 | y | 0 - 0 - 1.25
4 | 3 | 4 | x | 0.96 - 0 - (-0.054)
5 | 4 | 5 | y | 0.54 - 0 - 0
6 | 5 | 6 | x | 0.193 - 0 - 0
gripper | 6 | gripper | - | 0.11 - 0 - 0

DH-parameters
i | T | alpha_(i-1) | a_(i-1) | d_i | theta_i
--- | --- | --- | --- | --- | ---
1 | T_0_1 | 0 | 0 | 0.75 | theta_1
2 | T_1_2 | -pi/2 | 0.35 | 0 | theta_2 - pi/2
3 | T_2_3 | 0 | 1.25 | 0 | theta_3
4 | T_3_4 | -pi/2 | -0.054 | 1.5 | theta_4
5 | T_4_5 | pi/2 | 0 | 0 | theta_5
6 | T_5_6 | -pi/2 | 0 | 0 | theta_6
G | T_6_G | 0 | 0 | 0.303 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Joints 4, 5 and 6 of the KR210 are revolute. Their axes intersect at a single point, the wrist center at joint 5. So the inverse kinematics problem is decoupled into
- the position problem for the wrist center and
- the orientation problem for the end effector.

Inverse Position
- theta1: It is based on the X, and Y components of the wrist center on base frame.

        theta1 = atan2(WC[1], WC[0])

- theta2:
  - We need to know the angle b/w the wrist center and the x-axis of joint 2 - angle_a.
  - Plus, we need to know the phi2 formed by joint2-joint3-line and joint2-joint5-line.
        
        phi2 = atan2(WC[2] - 0.75 sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)

        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
        side_c = 1.25
        angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/(2*side_b*side_c))

        theta2 = pi/2 - angle_a - phi2

- theta3:
  - It is straight-forward to

        angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b)/(2*side_a*side_c))

        theta3 = pi/2 - (angle_b + 0.036)

Inverse Orientation
-  The orientation of the end-effector, the joint variables 4, 5 and 6 are the Euler angles of the rotation R3_6

        R3_6 = R0_3.inv("LU") * ROT_EE

        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code is implemented as the above derivation. The arm is successfully picking up blue object and drop it to the bin. Belows are screen shot of the run

![Screen shot of arm picking up object][image4]

![Screen shot of arm dropping object][image5]

