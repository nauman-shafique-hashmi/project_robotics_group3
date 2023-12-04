# !/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import sys
import rospy

rospy.init_node('niryo_blockly_interpreted_code')
n = NiryoRosWrapper()

n.calibrate_auto()

try:
   # 1
   # 3
   # 3

   # 4

   # Move to an observation position then
   # Try to do a vision pick:
   if n.vision_pick_w_obs_pose('group3', 0/1000.0, ObjectShape.ANY, ObjectColor.ANY, [-0.004, -0.155, 0.329, -0.09, 1.449, -1.7])[0]:
     # If an object has been taken, do:
     n.place_from_pose(*[0.375, 0.028, 0.348, -0.021, 1.176, 0.015])
   n.move_pose(*[0.134, 0.009, 0.329, -0.29, 1.456, -0.097])

except NiryoRosWrapperException as e:
   sys.stderr.write(str(e))