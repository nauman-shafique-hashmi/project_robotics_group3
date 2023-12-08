
# Turtlebot3 and Niryo Ned2 Working together
### Group3:  ADEWALE Atanda, Nauman Shafique HASHMI,  Qian Zhiling

### Intro/Project Abstract:
The autonomous driving meaning enabling robot drive itself without any external intervention, is the future of industrialization. In the current era of industrial automation where many many of the jobs are replaced by the autonomous robots describes its need to learn about it . In this practical project, We did the configuration and parameter tuning of the Turtlebot3 Burger based on ROS in order to detect lanes, which is an essential part of a autonomous driving robot. The project mainly covers the following things: camera intrinsic calibration, camera extrinsic calibration, parameter tuning for lane detection and autonomous driving based on lane detection. It also includes configuring Niryo Ned 2 with the local network used for our remote PC and SBC (TurtleBot3) and establishing a communication link among them using ROS_MASTER_URI so that Niryo Ned2 can pick an object from his workspace and place it on top of the TurtleBot3 whenever he asks for it. 

## Technologies 
The key technologies used in this project can be divided into four parts: environment perception, behavior decision-making, path planning and motion control. Let's briefly introduce the composition of autonomous driving technology from both hardware and software aspects
### Reason to Use a Camera for Autonomous Driving with TurtleBot3:
The TurtleBot3 provided by the lab comes with both LIDAR and the camera. But the reason to use a Camera for Autonomous Driving with TurtleBot3 was quite obvious. Because the problem in hand is an Image-based perception task where we need color and texture Information which is used in object detection such as lane detection and ArUco marker detection. Which is not possible with LIDAR that primarily provides distance and 3D positional information only.


## Required Libraries and Packages to Start
      ros-noetic-image-transport 
      ros-noetic-cv-bridge 
      ros-noetic-vision-opencv 
      opencv 
      libopencv-dev 
      ros-noetic-image-proc
      Autorace package

## First Step Installing the Required Packages .
For this step we have followed the instructions provided on [emanual robotics][1].
Install the AutoRace 2020 meta package on Remote PC.

      cd ~/catkin_ws/src/
      git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
      cd ~/catkin_ws && catkin_make
            
Install additional dependent packages on Remote PC.

      sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc

## Connecting to Turtlebot3

Connecting to turtlebot over ssh with password napelturbot

      ssh ubuntu@192.168.0.200
      
 This is very important to bringup the turtlebot before running any node related to camera to robot control. 
 
Bring Up Turtlebot on PI.

     roslaunch turtlebot3_bringup turtlebot3_robot.launch 

## Camera Calibaration

For Camera Callibration we have done the same proccedure as listed below. We used the autorace camera package to get the undistorted image so we can perform the lane detection.

Launch roscore on Remote PC.
      
      roscore
Trigger the camera on SBC.
      
      roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

### Intrinsic Camera Calibration
After launching the camera node from raspberry pi we can now run the following package on remote pc to do the camera calibration. We used the checkboard pattern to do this task. 

      roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=calibration
      
One of the multiple pictures taken during calibration proccess is listed below.

![Calibartion](images/left-0033.png)

After completing the callibration we saved the data. By default calibrationdata.tar.gz is created at /tmp folder on remote pc. This compressed folder contain the sample images as well as the callibration data in a file named ost.yaml. For using the Autorace Camera package we copied the calibration file data and pasted into the file present in the Autorace Race package named [camerav2_320x240_30fps.yaml](/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml) 

### Extrinsic Camera Callibration

Open a new terminal and launch the intrinsic camera calibration node.
      
      roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
Open a new terminal and launch the extrinsic camera calibration node.

      roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration

Now to set the region of interest and the warping parameters for projected image we can run the following command in new terminal.
      
      rosrun rqt_reconfigure rqt_reconfigure

After the extrinsic callibration the results are stored in  turtlebot3_autorace_camera/calibration/extrinsic_calibration/ having two files compensation.yaml and projection.yaml. We copied those files to our package so we can use them to republish the topic coming from raspberry pi after applying the desired projection and compensation. These file are located at [Extrinsic Calibration Files](/calibration/extrinsic_calibration)

## ROS Package
Overall structure of the ros packge is shown below.
 -section to be updated 

### Lane Detection
 -section to be updated 

### Lane Detection Results

Right Lane Detected.
![Right Lane Detected](images/right.png)

Left Lane Detected.
![Left Lane Detected](images/left.png)

Both Lanes Detected.
![Both Lanes Detected](images/both.png)

### Control Lane
        
 -section to be updated 

### To Run this code


To Run the code we need to do following steps on remote PC and Turtlebot.

- Run ros master on PC. 

      roscore
 
- Connecting to Turtlebot3

  Connecting to turtlebot over ssh with password napelturbot

      ssh ubuntu@192.168.0.200
      
- Bring Up Turtlebot on PI.

     roslaunch turtlebot3_bringup turtlebot3_robot.launch 

-Start Capturing from Camera.

     roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
     
     
- In another termianl supply intrinsic parameters to the camera using .
      roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action

- In new termianl supply extrinsic parameters to the camera using .
      roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
- To view the scene in another terminal type.

      rqt

The results can be viewed on the topics.

      /camera/image/compressed
      /camera/image_projected/compressed/ 
Lane Filtering results on 
      
      /detect/image_yellow_lane_marker/compressed
      /detect/image_white_lane_marker/compressed
Filtered  lanes detection View on 

      /detect/image_lane/compressed



## Demo and Trial Videos

**Previous Trial Video.

<p float="middle">
  <img src="images/tb_moving.gif"/>
</p>

**Complete Run Video.

<p float="middle">
  <img src="images/tb_moving.gif"/>
</p>

[1]:https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving
[2]: https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/
