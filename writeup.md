## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace. (I chose to fork the repo and clone my personal repo instead)
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)
[image1]: ./misc_images/jointAnglesAxes.png
[image2]: ./misc_images/kr210_links.png
[image3]: ./misc_images/general_formula.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

![alt text][image2]

These images are from the lessons, and I used to as a starting point for finding the parameters.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | q1
1->2  | - pi/2     | 0.35   | 0      | -pi/2 + q2
2->3  | 0          | 1.25   | 0      | q3
3->4  |  - pi/2    | -0.054 | 1.5    | q4
4->5  | pi/2       | 0      | 0      | q5
5->6  | - pi/2     | 0      | 0      | q6
6->EE | 0          | 0      | 0.303  | 0

The table was created by starting with the joint locations (as suggested from the lessons). The parameters were derived from the UDRF file describing the arm for Gazebo.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I used the table above and and the general formula shown under, to created the individual transformation matrices.

![alt text][image3]

This resulted in the following transformation matrices:

##### T0_1
||||
---|---|---|---|
cos(q1) | -sin(q1) | 0 | 0
sin(q1) | con(q1) | 0 | 0
0 | 0 | 1 | 0.75
0 | 0 | 0 | 1

##### T1_2
||||
---|---|---|---|
sin(q2) | cos(q2) | 0 | 0.35
0 | 0 | 1 | 0
cos(q2) | -sin(q2) | 0 | 0
0 | 0 | 0 | 1

##### T2_3
||||
---|---|---|---|
cos(q3) | -sin(q3) | 0 | 1.25
sin(q3) | cos(q3) | 0 | 0
0 | 0 | 1 | 0
0 | 0 | 0 | 1


##### T3_4
||||
---|---|---|---|
cos(q4) | -sin(q4) | 0 | 0
0 | 0 | 1 | 1.5
-sin(q4) | -cos(q4) | 0 | 0
0 | 0 | 0 | 1

##### T4_5
||||
---|---|---|---|
cos(q5) | -sin(q5) | 0 | 0
0 | 0 | -1 | 0
sin(q5) | cos(q5) | 0 | 0
0 | 0 | 0 | 1


##### T5_6
||||
---|---|---|---|
cos(q6) | -sin(q6) | 0 | 0
0 | 0 | 1 | 0
-sin(q6) | -cos(q6) | 0 | 0
0 | 0 | 0 | 1


##### T6_EE
||||
---|---|---|---|
1 | 0 | 0 | 0
0 | 1 | 0 | 0
0 | 0 | 1 | 0.303
0 | 0 | 0 | 1

To get the pose to end-effector we simply multiply these matrices:

```T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE```

To correct to rotation, I also used the following matrices:

##### ROT X
|||
---|---|---|
1 | 0 | 0
0 | cos(pi/2) | -sin(pi/2)
0 | sin(pi/2) | cos(pi/2)

##### ROT Y
|||
---|---|---|
cos(pi/2) | 0 | sin(pi/2)
0 | 1 | 0
-sin(pi/2) | 0 | cos(pi/2)

##### ROT Z
|||
---|---|---|
cos(pi/2) | -sin(pi/2) | 0
sin(pi/2) | cos(pi/2) | 0
0 | 0 | 1

This was done to compensate for actual angel of the end-effector.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The problem can be broken into two sub-problems: finding the wrist center and finding the orientation.

For each calculation iteration, ROS feeds IK server the end effector position and orientation relative to the base frame.

These values are used to find the wrist center, which in turn is used to find the different joint angles (thetaX).



TODO

```
ee_target = Matrix([px, py, pz])





With theta1, theta2, and theta3 computed, we can use FK to compute a rotation matrix for links 0 to 3. Using roll, pitch, and yaw values we can also compute the gripper rotation. These two are then used to compute a gripper rotation matrix relative to link3.

My code looks like this:

```
R_03 = self.T[(0,3)][:3,:3].evalf(subs =
                                    {self.q1: theta1,
                                    self.q2: theta2,
                                    self.q3: theta3})

R_36 = R_03.T*Rrpy
theta4 = atan2(R_36[2,2], -R_36[0,2])
theta5 = acos(R_36[1,2])#atan2((R_36[0,2]**2 + R_36[2,2]**2)**0.5, R_36[1,2])
theta6 = atan2(-R_36[1,1], R_36[1,0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

I decided to extract the calculation to a separate file and class to improve testability. That way I could reuse the same code for the debug-part as well as the IK server. I originally went for the normal implementation without a class, but I check out what other students had done, and I discovered this approach, and decided to use it as well.

I have tried to structure the code for both readability and speed. Most of the operations are because of this, broken out as separate function calls.

Speed is crucial for this IK server, so my code have certain optimizations:
1. Initialize the sympy matrices only once.
2. Avoid using sympy simplify() function.
3. Reduce matrix multiplications where possible.

##### Possible improvements
I believe even more speed could be achieved if we could avoid sympy completely. This comes at the price of readability, and should only be done if speed is really needed.

Another improvement is that my implementations does a lot of rotations of the gripper as it moves. I never managed to nail down why this happens, but of course the implementation would improve a lot if this can be avoided. To achieve this fully, I guess the calculation needs to compensate for the previous position, and find the movement with the least rotations.
