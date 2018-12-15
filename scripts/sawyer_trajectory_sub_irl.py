#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('ssg')
from std_msgs.msg import String
from intera_interface import Limb
from intera_motion_msgs.msg import (
    Trajectory,
    TrajectoryOptions
)
from intera_motion_interface import (
    MotionTrajectory,
)
from time import time

def move_arm(traj_msg):
    traj = MotionTrajectory()
    traj.set_data(traj_msg)
    rospy.logwarn(time())
    rospy.logwarn(len(traj.to_dict()['waypoints']))
    traj.send_trajectory(wait_for_result=False, timeout=1.0)

    
rospy.init_node('sawyer_kinect_listener', anonymous=True)
rospy.Subscriber('/ssg_trajectory', Trajectory, move_arm)
rospy.spin()
