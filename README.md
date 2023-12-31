# ROSBot Harmony 1.0
Turtlebot3 and Niryo-Ned2 Working together

ViBOT_2023/Group3: Nauman Shafique HASHMI, Atanda Abdullahi Adewale and Qian Zhiling

## Intro/Project Abstract:

<p>
The autonomous driving meaning enabling robot drive itself without any external intervention, is the future of industrialization. In the current era of industrial automation where many many of the jobs are replaced by the autonomous robots describes its importance in robotics and a need for a study. 
</p>
<p>
The objective of this project is to detect lane (yellow and white line in an unstable light condition) perform autonomous lane driving of turtlebot3 (TB3) robot and to establish communication a Niryo Ned2 robot in a ROS (Robot Operating System) environment. 
</p> 
<p>
The project mainly covers the following things: camera intrinsic calibration, camera extrinsic calibration, Camera HSL parameter tuning for lane detection
TurtleBot3 is then tasked with moving until it detects a QR ArUco tag, at which point it communicates with the Niryo Ned2 robot through ROS messages. The Niryo Ned2 then performs certain actions {pick and place an object on TB3} and communicates back to the TurtleBot3 when done, allowing it to resume its movement
</p>
<p>
To enable communication between the PC, acting as the ROS master, and TurtleBot3 as well as Niryo, the ROS_MASTER_URI and ROS_IP parameters on both robots are set to the PC's IP address to form a ROS network group, facilitating the exchange of information through ROS topics
<p> 

# Part 1:
## Camera Calibration:

No camera is 100% perfect, each comes with inherent imperfections, often stemming from aberrations caused by faulty lenses or assembly defects in camera parts. Primarily, lens-related distortions manifest as radial and tangential distortions. Radial distortions, namely Barrel and Pincushion distortions, are prevalent. These deviations hinder the preservation of straight or parallel lines, causing them to seemingly converge towards infinity, termed as the Vanishing Point.
Barrel distortion causes parallel lines to curve outward, while Pincushion distortion makes parallel lines appear to curve inward. But there are some cameras which are deliberately made with those distortions. Such as cameras with Fish eye lenses. And the idea behind is that we can obtain a wider field of view with them. One of such camera is used in our project equipped with the following specifications: 
- Raspberry Pi Fish-eye Lens
- FOV: 160 degrees Diagonally
- Focal Length: 3.15mm

## Maths:
 Apart from using AutoRace package that ease the way for us doing Camera Calibration, The concepts of Linear algebra played an important role understanding the calibration.
- Principles of Camera Imaging: 
![Qian!](/images/1.png "bla")
Step 1: OpenCV did Harris corner detection first

Step 2: Solve the product of the intrinsic and extrinsic matrices: The product is a homography matrix
![Qian!](/images/2.png "bla")

We set the world coordinate on the check board, so here R1 R2 means the first and second column of the matrix.
![Qian!](/images/3.png "homography matrix")

![Qian!](/images/4.png "bla")

Step 3 solve intrinsic matrix:

![Qian!](/images/5.png "bla")

Consider

![Qian!](/images/6.png "bla")

![Qian!](/images/7.png "bla")

Notate A as :

![Qian!](/images/8.png "bla")

![Qian!](/images/9.png "bla")

So we have: 

![Qian!](/images/10.png "bla")

To solve out B, according to:

![Qian!](/images/11.png "bla")

## Intrinsic Callibration Results:

<div style="display: flex; justify-content: center;">
    <figure style="margin-right: 20px;">
        <img src="images/before_intrinsic_calibration.png" alt="Image 1" style="width: 300px; height: auto;" />
        <figcaption>Before Calibration</figcaption>
    </figure>
    <figure>
        <img src="images/after_intrinsic_calibration.png" alt="Image 2" style="width: 300px; height: auto;" />
        <figcaption>After Calibration</figcaption>
    </figure>  
</div>


> This picture for intrinsic calibration taken from emanual.robotis.com for our explanation purposes. We had exactly the same result but missed to take picture.


## Extrinsic Calibration
solve extrinsic matrix through

![Qian!](/images/12.png "bla")

Finally, we need to mention that the automatic calibration tools considered tackling radial distortion:

![Qian!](/images/13.png "bla")

And tangential distortion:

![Qian!](/images/14.png "bla")

## Extrinsic Callibration Results:

<figure style="margin-bottom: 20px;">
    <img src="/images/extrinsic_calibration.png" alt="Image 1" style="width: 450px; height: auto;" />
    <figcaption style="text-align: center;">Camera Perspective</figcaption>
</figure>
<figure>
    <img src="/images/bird_eye_view.png" alt="Image 2" style="width: 400px; height: auto;" />
    <figcaption style="text-align: center;">Bird's eye view</figcaption>
</figure>



# Part 2:

## Lane Detection 
<p>
Once the calibraiton process is done, we are good to go for lane detection. At this point, creating an optimal lighting environment is pivotal for effective lane detection in TurtleBot3 operations. Because variations in luminance across the track present a significant challenge, with certain sections experiencing excessive light resulting in glare, while others remain comparatively darker. 
</p>

<p>
This discrepancy necessitates individualized HSL parameter configurations rather than a universal setting in the robotics lab for every lighting condition. Extensive trials involving diverse light combinations were conducted in an attempt to balance the illumination across all track sections, yet none proved successful. However, barring the use of dual projection lights. These lights provided the flexibility to adjust orientation and intensity, enabling uniform illumination overthe entire track.
For tuning the camera parameters We operated on 5600k temperature with 100% intensity value.
</p>

### Lane Detection Algorithm:


Algorithm Steps

      Thresholding
      Apply Perspective Transformation to Get a Bird’s Eye View
      Identify Lane Line Pixels
      Line Fitting thorugh the White piexels detected 
      Set Sliding Windows for White Pixel Detection
      Overlay Lane Lines on Original Image
      Display Final Image

### Lane Detection Results
![Qian!](/images/detected_lanes.png "bla")

- <b>Algorithm Steps</b>
<p>
Autorace package detect the lanes (yellow and white) using thresholding. The thresholding is performdef cbFollowLane(self, desired_center):
         if not self.stopped:
             self.ids = None
             center = desired_center.data
 
             error = center - 500
 
             Kp = 0.0025
             Kd = 0.007
 
             angular_z = Kp * error + Kd * (error - self.lastError)
             self.lastError = error
 
             self.twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
             self.twist.linear.y = 0
             self.twist.linear.z = 0
             self.twist.angular.x = 0
             self.twist.angular.y = 0
             self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
             self.pub_cmd_vel.publish(self.twist)ed based on the low and high HSL values of both the lanes. Every pixel value below the lower threshold replaced with  ‘0’ black pixel and very pixel value above the thresholding is turned to ‘1’ white pixel, thus resulting in a binary image. Now that we have binary images for both the lanes, the next step is creating a masks of those lanes which serve as ROI for further processing.
After having the masked values, bitwise AND operations is performed between the original HSL image and masked images, this will result in filtering out each colored lane.
</p>


     def maskWhiteLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        Hue_l = self.hue_white_l
        Hue_h = self.hue_white_h
        Saturation_l = self.saturation_white_l
        Saturation_h = self.saturation_white_h
        Lightness_l = self.lightness_white_l
        Lightness_h = self.lightness_white_h

        # define range of white color in HSV
        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)

        if self.is_calibration_mode == False:
            if fraction_num > 35000:
                if self.lightness_white_l < 250:
                    self.lightness_white_l += 5
            elif fraction_num < 5000:
                if self.lightness_white_l > 50:
                    self.lightness_white_l -= 5

        how_much_short = 0

        for i in range(0, 600):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= 100:
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes white lane filtered image in compressed type
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes white lane filtered image in raw type
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, "bgr8"))

        return fraction_num, mask
        
<p>
    
The next step is marking those filtered lanes. AutoRace does this using two methods:
- Fitting a second order polynomial line in the detected lanes using the existing coefficient of the lane
- using sliding window method. 
</p>

    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
        nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

        # Again, extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        lane_fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
            
        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20

        # Set height of windows
        window_height = np.int(img_w.shape[0] / nwindows)

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated for each window
        x_current = lane_base

        # Set the width of the windows +/- margin
        margin = 50

        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Create empty lists to receive lane pixel indices
        lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                nonzerox < win_x_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_lane_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_lane_inds) > minpix:
                x_current = np.int(np.mean(nonzerox[good_lane_inds]))

        # Concatenate the arrays of indices
        lane_inds = np.concatenate(lane_inds)
def cbFollowLane(self, desired_center):
         if not self.stopped:
             self.ids = None
             center = desired_center.data
 
             error = center - 500
 
             Kp = 0.0025
             Kd = 0.007
 
             angular_z = Kp * error + Kd * (error - self.lastError)
             self.lastError = error
 
             self.twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
             self.twist.linear.y = 0
             self.twist.linear.z = 0
             self.twist.angular.x = 0
             self.twist.angular.y = 0
             self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
             self.pub_cmd_vel.publish(self.twist)
        # Extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except:
            lane_fit = self.lane_fit_bef
def cbFollowLane(self, desired_center):
         if not self.stopped:
             self.ids = None
             center = desired_center.data
 
             error = center - 500
 
             Kp = 0.0025
             Kd = 0.007
 
             angular_z = Kp * error + Kd * (error - self.lastError)
             self.lastError = error
 
             self.twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
             self.twist.linear.y = 0
             self.twist.linear.z = 0
             self.twist.angular.x = 0
             self.twist.angular.y = 0
             self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
             self.pub_cmd_vel.publish(self.twist)
        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

# Part 3:
## Lane Following 

<p>
The official control line code used a pd controller to follow the lines. Firstly, the callback function below listen to the topic '/control/lane'. The topic will provide summation of all the x coordinates of the detected line points , which is called “desired_center” in code below. Then the following code will calculate the error between tdef cbFollowLane(self, desired_center):
         if not self.stopped:
             self.ids = None
             center = desired_center.data
 
             error = center - 500
 
             Kp = 0.0025
             Kd = 0.007
 
             angular_z = Kp * error + Kd * (error - self.lastError)
             self.lastError = error
 
             self.twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
             self.twist.linear.y = 0
             self.twist.linear.z = 0
             self.twist.angular.x = 0
             self.twist.angular.y = 0
             self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
             self.pub_cmd_vel.publish(self.twist)he “desired_center” and the current image center 500.Then use this error to do the pd control. 
</p>

     def cbFollowLane(self, desired_center):
             if not self.stopped:
                 self.ids = None
                 center = desired_center.data
     
                 error = center - 500
     
                 Kp = 0.0025
                 Kd = 0.007
     
                 angular_z = Kp * error + Kd * (error - self.lastError)
                 self.lastError = error
     
                 self.twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
                 self.twist.linear.y = 0
                 self.twist.linear.z = 0
                 self.twist.angular.x = 0
                 self.twist.angular.y = 0
                 self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
                 self.pub_cmd_vel.publish(self.twist)
                 

 <p>
  We redesigned the control line node in autorace package. We added a subscriber to get compressed image from topic '/camera/image/compressed' ：
 </p>
 
         self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)

## Aruco Marker Detection
<p>
 When the camera detected the aruco , the turtlebot3 stopped until the it hear the message “finished” from the topic “/channel_turtle_niryo"
</p>

def image_callback(self, msg):
        rospy.loginfo("Seen an image")
        
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers if the detector is enabled
            if self.detector_enabled:
                parameters = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)
                self.corners, self.ids, self.rejectedImgPoints = detector.detectMarkers(gray)
                #print(self.corners)
                rvec, tvec, _ = self.my_estimatePoseSingleMarkers(self.corners, self.markerLength, self.camera_matrix, self.dist_coeffs)
                
                if self.ids is not None and len(self.ids) > 0 and tvec[0][-1]<=self.distancethreshold:
                    print('aruco distance is',tvec[0][-1])
                    rospy.loginfo("ArUco detected!")
                    self.stop_turtlebot()
                    self.counter = rospy.Time.now()  # Update the class attribute counter
                    self.detector_enabled = False  # Disable the detector

        except Exception as e:
            rospy.logerr(e)

        # Check the counter status and resume if necessary
        if self.counter is not None and (rospy.Time.now() - self.counter) > self.interval_duration:
            self.resume_turtlebot()

   ## Distance calculation from the ArUco Marker
   
<p>

Because we used newest version of opencv, so we cannot directly use package to get the aruco to camera translation. So we defined our own function. It solved a pnp problem .we set the world coordinate in the middle of aruco, that’s why all the real world marker_points has 0 z coordinates. We took the “Infinitesimal Plane-Based Pose Estimation” method to solve the problem.

</p>


     def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
      
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

 ## Communication between niryo and turtlebot :

<p>
 We defined a pair of subscriber and publisher in both the turtlebot node and niryo node for communication in turtlebot node
</p> 

         self.sub_turtle = rospy.Subscriber('/channel_turtle_niryo', Connectniryo, self.turtleCallBack, queue_size = 1)
         self.pub_turtle = rospy.Publisher('/channel_turtle_niryo', Connectniryo, queue_size=1)
         self.msg_turtle_ned = Connectniryo()

 <p>
  In niryo node 
 </p>
 
        self.sub_niryo = rospy.Subscriber('/channel_turtle_niryo', Connectniryo, self.niryoCallBack, queue_size = 1)
        self.pub_niryo = rospy.Publisher('/channel_turtle_niryo', Connectniryo, queue_size=10)


## Our custom message:

<p>
When turtlebot detect the aruco, it will use publisher pub_turtle to publish “detected = True,  finished = False” , when niryo finishing picking, it will use publisher pub_niryo to publish “detected = False, finished =True”.
</p>

<h1><b>PART 4</b></h1>

## Communication

This part demonstrates the interaction between two robots: Turtlebot3 and Niryo Ned2. The Turtlebot3 sends a message to the Niryo Ned2 robot indicating that it has stopped. The Niryo Ned2 robot then proceeds to perform a vision pick, place the object, and return to the initial pose. TB3 resumes self driving.
The communication between the two robots is facilitated through a ROS topic called "channel_turtle_niryo".

Network Config:
on Ubuntu, edit <b>/etc/network/interfaces </b>
Set the IPv4 Address to 192.168.0.100 on PC, 192.168.0.200 on TB3 and 192.168.0.150 on Niryo Ned2

Then set network group:
<pre>
    export ROS_MASTER_URI=http://192.168.0.100:11311
    export ROS_IP=192.168.0.100
</pre>
on Pc, repeat on TB3 and Niryo, set their IP respectively. ping the PC or telnet 192.168.0.100 11311 from each robot toconfirm connection.   

Create a packeage <b>robot_com</b> with a node <b>ned2.py</b>, make executable with <pre> chmod +x ned2.py </pre>
make a msg dir, for custom message below   

Connectniryo.msg:
A custom ROS message type used for communication between TurtleBot3 and Niryo Ned2.
<pre>
    bool detected
    bool finished
</pre>

dont forget to build the pacakge in the workspace directory i.e <pre>catkin build </pre>  

# To Run this code

To Run the code we need to do following steps on remote PC and Turtlebot.
 
 - Run ros master on PC. 
      roscore
 
 - Connecting to Turtlebot3
   
      Connecting to turtlebot over ssh with password napelturbot
   
          ssh ubuntu@192.168.0.200
     
 - Bring Up Turtlebot on SBC.
   
       roslaunch turtlebot3_bringup turtlebot3_robot.launch
 
 - In new terminal trigger the camera on SBC
   
      roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
 
 -Run  intrinsic camera calibration launch file on Remote PC
 
      roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
 
 -Run extrinsic camera calibration launch file on Remote PC
 
      roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
 
 - Open terminal and Run a lane detection launch file on Remote PC
   
      roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action
   
 - To be able to communicate with Niryo, also run this commnad
   
       rosrun rob_com ned2
   
 - Check if the results come out correctly, Open z nds terminal and use the command on remote PC
 
      roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
 
 - In another terminal open 
 
      rqt
 
 The results can be viewed on the topics.
 
     /camera/image/compressed
     /camera/image_projected/compressed/ 
     
 Thresholding results on 
     
     /camera/mask_lane_detected/compressed
     
 Final lane detection on 
 
     /camera/midlane_detected/compressed
 
    - After using the commands, TurtleBot3 will start to run.
 
 
 
 ## Demo and Trial Videos
 
 **Previous Trial Video.
 
 <p float="middle">
 <img src="/images/tb_moving.gif"/>
 </p>
 
 **ArUco Market detection.
 
 <p float="middle">
 <img src="/images/ArUco_detection.gif"/>
 </p>

 **Niryo Failed Attempt to Work 
  <p float="middle">
 <img src="/images/ned2_working.gif"/>
 </p>
 
 [1]:https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving
 [2]: https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/

