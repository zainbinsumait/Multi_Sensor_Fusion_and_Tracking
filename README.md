![image](https://user-images.githubusercontent.com/76461363/207363584-e22868d0-7057-4d82-beb3-d6475262d525.png)


## Summary

Multi-sensor fusion and tracking is the process of combining information from multiple sensors to provide a more accurate, reliable, and comprehensive view of a situation or environment. This can be done in real-time, allowing for the tracking of moving objects and the identification of potential threats or obstacles.

In the context of robotics, multi-sensor fusion and tracking can be used to improve navigation and decision-making. For example, a robot equipped with a camera, lidar, and radar sensors can use information from all of these sensors to create a detailed map of its surroundings and track the movement of objects within that environment. This can enable the robot to navigate around obstacles, avoid collisions, and make more informed decisions about its actions.

Multi-sensor fusion and tracking can also be used in other applications, such as surveillance, autonomous vehicles, and aircraft collision avoidance. By combining data from multiple sensors, it is possible to create a more complete picture of a situation and make more accurate predictions about the behavior of objects within that environment. 

During this project, a turtlebot3 was used and a monocular camera. The robot get the information from the camera (through a computer) to reach a specific target with and without obstacles. **This project was done during the last year of ESIREM Robotics program ( Engineering school in Dijon city - France).**

Teammates : 
* Zain BIN SUMAIT
* Mehdi HAHOU
* Steve Alex TAMO

# TABLES OF CONTENTS:


 1. [Introduction](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#introduction)

 2. [Camera Calibration](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#camera-calibration)

 3. [Detection and pose estimation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#detection-and-pose-estimation)

   1) [ Approximation 2D without pose estimation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-approximation-2d-without-pose-estimation)

   2) [3D pose estimation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-3d-pose-estimation)

 4. [Frame transformation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#frame-transformation)

 5. [Control system](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#control-system)

    * [Simulation to test the control system](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#control-system)

 6. [Integration of ROS ](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#integration-of-ros)

    1) [ What is ROS: ](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-what-is-ros)

    2) [ What is the publisher and Subscriber](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-what-is-the-publisher-and-subscriber)

    3) [Launch the Robot](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#3-launch-the-robot)

    4) [Launch the Camera](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#4-launch-the-camera)

 7. [Obstacle avoidance](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#obstacle-avoidance)

    1) [Obstacle detection](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-obstacle-detection)

    2) [The algorithm developed](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-the-algorithm-developed)

    3) [Similar RRT algorithm](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#3-similar-rrt-algorithm)

 8. [Conclusion](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#conclusion)
 9. [Demonstration](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#conclusion)
 




# Introduction:

During the Multi-Sensor Fusion and Tracking module we had a project to realize, the goal of this project is to control a 2-wheel mobile robot "Turtlubot3" under ROS melodic from an UEYE-camera using the configuration (Eye-to-hand):

*   **The first objective is to move the robot from an initial position to a Target position considering the target orientation (Parking).**

*   **The second objective is to do the same thing as task 1 but with obstacles, the task of the robot is to avoid these obstacles (in our case we used buckets of a red color) and reach the target.**

To solve our problem, we have divided the spots as objectives as can be seen in figure 1:

* **Camera calibration**

* **Pose initial and target robot estimation**

* **Control system**

* **Integration of ROS**

* **Obstacle avoidance**

![](https://user-images.githubusercontent.com/76461363/206848388-45c6d0bb-b566-4624-8acb-7dbebc14adbc.png)

![](https://user-images.githubusercontent.com/76461363/206848522-87638424-25fb-4ea0-b68d-5973452a468b.png)

# Camera Calibration:

Camera Calibration is the process of estimating the intrinsic and extrinsic parameters. Intrinsic parameters refer to the internal characteristics of the camera, such as focal length, tilt, distortion, and image center. The extrinsic parameters describe the position and its orientation in the real (X, Y, Z) frame.

**This is the first step we must take before doing anything else.**

![image](https://user-images.githubusercontent.com/43727159/206852163-db5e23dd-d242-49f8-90c0-682c7b1c17fe.png)

**The intrinsic parameters are:**

![image](https://user-images.githubusercontent.com/43727159/206852180-c4432ae5-21d3-4d3b-82b4-fb54dc0b307b.png)

$f$: the focal length.

$Ku, Kv$: the image magnification factors.

$Cu, Cv$: the coordinates of the projection of the optical center of the camera on the image plane.

$Suv$: which reflects the potential non-orthogonality of rows and columns of electronic cells that make up the camera sensor. Most of the time, this parameter is neglected and therefore takes a zero value.

**The extrinsic parameters are:**

![image](https://user-images.githubusercontent.com/43727159/206852295-a13fe862-3d15-4ae4-93f8-12f7f6c37818.png)

$R3x3$: which is the rotation matrix allowing to pass from the reference frame linked to the working space to the reference frame linked to the camera.

$tx, ty, tz$: which are the components of the translation vector allowing to pass from the reference frame linked to the working space to the reference frame linked to the camera.

In total, there are 12 parameters to be estimated (the rotation matrix R contains 9 elements, but the relation that defines it as a rotation matrix **R. RT = 1** reduces the number of independent elements to 3: the 3 polar angles).

**Example of camera calibration (Distortion Elimination):**
![](https://user-images.githubusercontent.com/76461363/206848758-15ceecfe-e786-4db3-91c4-67c49b81de3f.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image014.png)

**Code Source:**

we calibrate the camera with two methods, the first one we build a code using some function of OpenCV in python:

We have used a chessboard 7x5 for the calibration.

```python
#To Find the chess board corners
ret, corners = cv.findChessboardCorners(gray, (7,5), None)

```

we define Criteria (the maximum number of iterations and/or the desired accuracy), we use it for Detecting corner’s location in subpixels, in our case we made:

```python
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
```

To generate the camera calibration there is a function of OpenCV **CalibrateCamera**, where we pass on it the object point (7*5x3), and the corners of the images contain the chessboard.!
```python
_, mtx, dist,_,_ = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
```

The function generates for us the camera matrix and distortion parameters and the translations and rotations on each image.

**The results:**

we find the following results for our camera; we save it in a yaml file using the command:

![](https://user-images.githubusercontent.com/76461363/206848928-91125865-05bf-4027-9433-5eae101e58d4.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image019.png)

We save these parameters in a yaml file using this command:

```python
yaml.dump(cal_data,f)
```

**2nd Method:**

There are several methods to calibrate the camera more accurately. Such us “the Single Camera Calibrator App” of MATLAB or also the packages of “camera_calibration” in Ros. In our case we used the second one because it’s easy to use and it takes the images of the chessboard autonomously.

And it generates a zip file with all the images taken during the calibration and also a .yaml file which contains the calibrated camera parameters.

To run this command in terminal:

```terminal
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.12 image:=/camera/image_raw camera:=/camera
```

**Result:**

we find the following results for our camera. ![](https://user-images.githubusercontent.com/76461363/206848934-9e2b89cc-abe8-433c-8ff3-188b27e97740.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image026.png)

# Detection and pose estimation

The objective of this step is to know the initial position of the robot and the target position to plan the trajectory.  A closed loop system based on visual servoing needs real time detection to minimize the error between the objective and the current position.

There are several methods to detect the pose using the camera for example Qrcodes, Aruco markers, or directly using image processing using some filters for detecting the depth of the image etc...

In our case we used an aruco marker attached to the top of the robot, we used the aruco library of OpenCV to simplify the detection. Then, two methods were tested to extract the correct position of the aruco as well as its orientation.

![](https://user-images.githubusercontent.com/76461363/206848967-f82fab51-d3bc-45bb-a5c6-13a9c39b9809.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image028.png)

## 1.    Approximation 2D without pose estimation

Here, we considered that our problem is always a 2D problem. The robot will move in x and y direction without considering the depth. In this approximation, the calibration matrix is not required. We directly detect the aruco markers in the camera using a simple python function given by OpenCV “aruco.detectMarkers” returns the position in the image (in pixel) of the 4 corners of the marker in order. We create a function called get_pose(img) in utils.py, this function includes the detectMarkers function, and it return the position of the center (x,y) of the marker and its orientation by calculating the orientation angle of the line between two points of the 4 corners of the marker  : arctan(Δy/Δx). After having the position and the orientation of the marker (current position of the robot and of the target), we calculate the transformation matrix for both positions. following this formula:

![](https://user-images.githubusercontent.com/76461363/206848977-c2432719-3bb4-43b4-9b8b-29c865d16f93.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image029.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image031.png)

## 2.    3D pose estimation
![image](https://user-images.githubusercontent.com/43727159/206852574-c65ec173-5885-4718-830b-26e7e0ff9fe9.png)

A real-world problem is a 3D problem, because of the real condition of the workspace. So, a 3D position is necessary to move the robot in an efficient way. To have this 3D position we can use another OpenCV function “estimatePoseSingleMarkers”

This function takes in input:  the corners of the marker, the marker size (in cm), the camera matrix and the distortion coefficients of the camera.  And it returns rotation vector and a translation vector (x,y,z) of a marker.

![](https://user-images.githubusercontent.com/76461363/206848984-defa72d0-ed6a-4153-bbb6-c538a01ccd5a.png)

Using this information, a transformation matrix can be calculated by computing the rotation matrix from the rotation vector (using Rodriguez function) and then build the transformation matrix.

Using two aruco markers with different ID, we can identify the current robot and the target position in real-time. This method has two advantages:

* **3D position, which allows to solve a 3D navigation.**

* **Detecting the target and the robot in real-time, which allows the robot to reach the target even if the target is moving (dynamic control).**

# Frame transformation

In this step, two transformation matrices are available, one for the current position, and the other for the target position in the camera frame. The objective is to get the target position in the robot frame, so we have all the coordinates and the angles in this frame.

![](https://user-images.githubusercontent.com/76461363/206848991-c9a72058-010f-4575-a4a0-cb51decc598e.png)

Supposing that: The transformation matrix from the target to the camera is **Tcamera_target**, and the transformation matrix to the robot is **Tcamera_robot**, so the Transformation matrix from the target to the robot:

![image](https://user-images.githubusercontent.com/43727159/206852789-316950e0-e986-4d2e-bbd3-5b4f7c92b31e.png)

To do so, we need to combine these two matrices by multiplying the inverse of the current transformation matrix of the robot by the target transformation matrix to receive the combined transformation matrix (t) which is called:

![](https://user-images.githubusercontent.com/76461363/206848993-bd079be2-cb68-4bbf-a315-ebc8a080d4b9.png)

In python we did that using the following code:

```python
np.linalg.inv(Robot_matrix_transformation).dot(Target_matrix_transformation)
```

# Control system

To move the TurtleBot, we need to send the order in a form of value of speed. 3 linear speeds and 3 rotational speeds in xyz axis. Corresponding to the number of degrees of freedom that we have on our robot (1 on x axis, 1 on y axis and 1 for the rotation around the z axis), two linear speeds (in respect of x and y) and one rotational speed (z) are given to the robot to move to a position. Consequently, we need to calculate these speeds first and then send it to the robot. The concept is to reduce the difference between the initial position and the target position. The difference includes the distance between them and the gap between their orientations.

**The distance between the two positions $\rho$:**

![image](https://user-images.githubusercontent.com/43727159/206851354-833e3395-a2a4-4cd7-bfd6-09090361df3e.png)

**The angle between the orientation of the robot and the target (θ):** 

![image](https://user-images.githubusercontent.com/43727159/206851373-d56e05a6-f90b-489f-82e1-c2c86e1de0d0.png)

**The angle between the orientation of the robot and the direction of the target (α):**

![image](https://user-images.githubusercontent.com/43727159/206851401-7e0e0b42-d556-40e2-93f2-6c6180617a1a.png)

![image](https://user-images.githubusercontent.com/43727159/206851419-c5ac66de-ff03-486e-b193-cd57df2c6800.png)

To reduce the distance, a forward movement toward the target is required. So, a forward speed and a direction must be calculated. Two parameters are responsible (α and $\rho$).
**The forward speed is:**

![image](https://user-images.githubusercontent.com/43727159/206851451-6562914c-1cd9-4727-a0b6-9080a9d5bf44.png)

**With the rotational speed of:**

![image](https://user-images.githubusercontent.com/43727159/206851464-54ad71da-0691-4e3a-8ffc-ec521ac12e9b.png)


To park the robot in the same orientation as the target, the third parameter (beta) should be reduced. For that we add another element to the rotational speed, and it becomes:

![image](https://user-images.githubusercontent.com/43727159/206851467-2df24690-850e-471e-96ea-b7304b39a096.png)


The speeds are calculated then sent every (1/50) seconds to the robot using ROS. To have a smooth movement, max speed should be determined which allows also to take in consideration the physical constraint of the vehicle.

## Simulation to test the control system

Simulate the control system using python code, by simulate a robot position which updates its position every time the speed is calculated. The next position is calculated by these equations:
![image](https://user-images.githubusercontent.com/43727159/206851571-051d74ed-12a9-472a-91eb-04fb504be304.png)


The operation is repeated in a loop until the distance becomes less than 0.01 unit.

With four different initial positions, [0,0], [20,0], [0,20], [20,20], and the initial orientation of 0° degree. The target position is [10,10] and the orientation of 180° degrees.

Gain values:![image](https://user-images.githubusercontent.com/43727159/206851586-fae147e2-7b3f-4d15-89e4-9b3d607762a7.png) we obtain the following trajectory graph. Noting that the gain gives importance to the parameter; giving 3.5 as a value for beta gives the parking task more importance than the 2.5 value (in absolute value).

![image](https://user-images.githubusercontent.com/43727159/206853185-fefb8433-1748-41a7-9bed-76a11054e9d8.png)


The trajectory obtained is not optimist for all the positions, which can be explained by the fact that in this simulation the robot is not taking on consideration it’s orientation correctly as the real world.

# Integration of ROS

## 1.    What is ROS:

ROS (Robot Operating System) is an open-source software development kit for robotics applications. ROS provides developers across all industries with a standard software platform that enables them to move from research and prototyping to deployment and production. ROS has a concept **Don't reinvent the wheel. Create new things and make them faster and better by building on ROS!**

## 2.    What is the publisher and Subscriber:

Message passing in ROS happens with the Publisher Subscriber Interface provided by ROS library functions.

A ROS Node can be a Publisher or a Subscriber. A Publisher is the one puts the messages of some standard Message Type to a particular Topic. The Subscriber on the other hand subscribes to the Topic so that it receives the messages whenever any message is published to the Topic.

Note that a publisher can publish to one or more Topic and a Subscriber can subscribe to one or more Topic. Also, publishers and subscribers are not aware of each other’s existence. The idea is to decouple the production of information from its consumption and all the IP addresses of various nodes are tracked by the ROS Master.

![image](https://user-images.githubusercontent.com/43727159/206851623-c98a45f6-8493-4060-8970-8de6c580491a.png)

## 3.    Launch the Robot:

The robot is already delivered with all the necessary ROS packages, otherwise you can easily get them by flowing a good tutorial made by ‘ROBOTIS’ [https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

To launch the robot in the terminal, write these commands:

```terminal
ssh ubuntu@192.168.0.200 “the password is napelturbot”
roslaunch turtlebot3_bringup turtlebot3_robot. launch
```

Till now we started these topics on our robot:

![image](https://user-images.githubusercontent.com/43727159/206853232-487c80a6-211c-46fa-8bfc-7a901e5772f9.png)


## 4.    Launch the Camera:

To publish our code in the robot, we must get the topics of the camera first; for that there is already Ros packages of the Ueye-camera by installing them from: [https://github.com/anqixu/ueye_cam](https://github.com/anqixu/ueye_cam)

```terminal
roslaunch ueye_cam rgb8.launch
```
## Move the robot

This step is to publish the speed calculated by publishing it on the robot’s topic “cmd_vel”. A servoing closed loop that gets the real time image, calculates the velocity, and sends it to the robot by a frequency of 50 Hz. The robot receives the instruction and goes to the target. The robot stops when the distance between the current and the target is about 40 pixels or (40/f)*Z = 5.6 cm (where f is the focal length and Z is the distance to the camera). And for the parking we added another condition to adjust the angle θ, so the robot has the same orientation as the target.

![image](https://user-images.githubusercontent.com/43727159/206853336-e69da6be-4a13-4a23-b34a-92463421cf93.png)

# Obstacle avoidance

Detect and avoid obstacles are important features for a mobile robot to navigate in an unknown environment. First step is detecting the obstacles where simple object detection or deep learning can be used. Second step is the path planning to reach the target.

Numerous of algorithm exist to determine the shortest path like A* and RRT. We chose to develop our own method to avoid obstacles as well as RRT algorithm.

## 1.    Obstacle detection

Detect an object considered as an obstacle needs a computer vision. Depending on the desired result, it can be a simple or a complex detection from shape or colour detection to a deep learning model to detect any object that can be considered as an obstacle. In this project we chose a simple colour detection with a red colour mask to extrait the red objects from the image.

![image](https://user-images.githubusercontent.com/43727159/206853397-ccc994a2-0279-43a7-b09d-fb7feb73d6d4.png)
## 2.    The algorithm developed

The input image is divided into squares with the same size as the robot in pixels, then we consider the square with red colour pixels as an obstacle square which can’t be added to the path. Next, for each box a cost is calculated by the following equations:

![image](https://user-images.githubusercontent.com/43727159/206853415-87f3b812-9225-42c3-b486-7873912d78cd.png)

Where d is the distance from the centre of the square to the target. 

![image](https://user-images.githubusercontent.com/43727159/206853431-29149875-283b-4f86-8cf0-eeaf49f3b155.png) 

if the square is an obstacle neighbour and n = 0 if not.

This equation returns a cost with an inverse relationship to the distance to the target with a bigger value to the obstacle’s neighbour squares, this gives a safety margin to completely avoid the obstacle.

The algorithm steps:

![image](https://user-images.githubusercontent.com/43727159/206853442-08476cc0-d84d-4d48-b3a4-ac85c7836d3f.png)

Applying this algorithm in real life gives us the result in the following figure. The path is as far as it can be from the obstacles to have a safe margin, however, it’s not exactly the optimist way. The algorithm can be improved to calculate the total cost of the path and then try another path as what other algorithms do (A star and RRT).  

The path points are then sent to the control system so that the robot reach each point as it’s finale target. Once it reached to the first point it goes to the second until the end of the path.

![image](https://user-images.githubusercontent.com/43727159/206853467-6e3b306d-c6be-4720-ba7c-3d055e46f74a.png)

## 3.    Similar RRT algorithm

Rapidly exploring Random Tree is an algorithm used in path planning to create a safety path for a trajectory.

Like the RRT process algorithm we try to describe a path that the robot can follow safety to join the target position avoiding the different obstacles.

The RRT algorithm follows the step below.

![image](https://user-images.githubusercontent.com/43727159/206853486-e88ef055-844d-4110-8124-dbd3a3866676.png)

In first time, around the start point we create a point by using a random vector generate by the goal position and the concept of nearest neighbour. Then we check if this point is not at the position of the obstacle and if it respects the condition due to its distance to the goal position. If this constraint is done, we add it in the path list and restart the process with this point as the start point now. In a small simulation we fixe an obstacle and create the path:

![image](https://user-images.githubusercontent.com/43727159/206853506-76d7e69e-8ef2-4619-a2e0-ee96e6068ce2.png)

In the image in green we have different points created by the algorithm. When the point is close to the obstacle, it creates a new point by respecting the constraint due to its distance from him until the robot arrives at the goal.

Now we apply it on an image. First with an algorithm for colour detection we detect the obstacle, and we extract its position. In this case the position is extracted in pixel.

![image](https://user-images.githubusercontent.com/43727159/206853539-8da94aca-7228-4bfb-ac5e-5a242c5a0504.png)


With this position and the robot and target position we can apply the algorithm to have the path that the robot can follow.

![image](https://user-images.githubusercontent.com/43727159/206853557-59bbc598-00c4-4643-9466-531fdf01ad2c.png)

# Conclusion

Controlling a robot in an unknown environment add more challenge to the control system and to the detection. Detecting the robot will not be as easy as while using the aruco marker. Moving the robot in a rugged terrain needs a robust control system that takes in consideration the tough surface and the sliding of the wheels. We can use a deep learning algorithm to detect the robot and the target. Using a CNN models to detect the robot without a marker in almost all the situation depending on how strong the model is.

The project allowed us to take on hand several important robotic skills, image processing, visual servoing, path planning, interpretation of the result, frame transformation, and soft skills as well.


# Demonstration

**Parking Without obstacles:**


https://user-images.githubusercontent.com/43727159/206853887-e0a0aa30-913d-4b7c-89eb-f56f0b196f90.mp4



https://user-images.githubusercontent.com/43727159/206853959-fd041a00-e6d3-49f9-926e-ec02b2816d9f.mp4



**Obstacles Avoidance:**




https://user-images.githubusercontent.com/43727159/206853936-a48d6ba8-54bc-4174-87c4-032a69ff4974.mp4



https://user-images.githubusercontent.com/43727159/206853965-2b47aea3-517e-421b-910f-23348d9536c1.mp4










