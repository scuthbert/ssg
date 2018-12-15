#!/usr/bin/env python

import roslib
roslib.load_manifest('ssg')
import rospy
from pyquaternion import Quaternion
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import (
    Trajectory,
    TrajectoryOptions
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from std_msgs.msg import Float64MultiArray
import tf
from intera_interface import Limb
import numpy as np

rospy.init_node('sawyer_collect_from_kinect', anonymous=True)

limb = Limb()

wpt_opts = MotionWaypointOptions(max_linear_speed=12.0,
                                    max_linear_accel=12.0,
                                    max_rotational_speed=12.0,
                                    max_rotational_accel=12.0,
                                    max_joint_speed_ratio=1.0)


traj_options = TrajectoryOptions()
traj_options.interpolation_type = TrajectoryOptions.CARTESIAN

orientation = Quaternion()

def update_hand(msg):
    print(msg)
    orientation = Quaternion(matrix=np.array(msg.data).reshape((3, 3)))

listener = tf.TransformListener()
hand = rospy.Subscriber('/leapmotion/arm', Float64MultiArray, update_hand)
pub = rospy.Publisher('ssg_trajectory', Trajectory, queue_size=10)

BASE_X = 0.5
BASE_Y = 0.0
BASE_Z = 0.3
OR_X = 1.0
OR_Y = 0.0
OR_Z = 0.0
OR_W = 0.0
SCALING = 0.6

pose_goal = Pose()

pose_goal.orientation.w = OR_W
pose_goal.orientation.x = OR_X
pose_goal.orientation.y = OR_Y
pose_goal.orientation.z = OR_Z
pose_goal.position.x = BASE_X
pose_goal.position.y = BASE_Y
pose_goal.position.z = BASE_Z

poseStamped = PoseStamped()
poseStamped.pose = pose_goal


traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

joint_angles = limb.joint_ordered_angles()
waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

traj.append_waypoint(waypoint.to_msg())
pub.publish(traj.to_msg())

traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

rate = rospy.Rate(30.0)
counter = 0
while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('kinect/user_1/torso', 'kinect/user_1/left_hand', rospy.Time(0))
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print 'excepted exception ' + str(type(e))
        rate.sleep()
        continue

    counter += 1
    rospy.loginfo( "setting goal state: " + str(trans))

    pose_goal = Pose()

    # pose_goal.orientation.x = orientation[1]
    # pose_goal.orientation.y = orientation[2]
    # pose_goal.orientation.z = orientation[3]
    # pose_goal.orientation.w = orientation.elements[0]

    pose_goal.orientation.w = OR_W
    pose_goal.orientation.x = OR_X
    pose_goal.orientation.y = OR_Y
    pose_goal.orientation.z = OR_Z

    trans = [-SCALING * trans[2] + BASE_X, -SCALING * trans[0] + BASE_Y, SCALING * trans[1] + BASE_Z]

    pose_goal.position.x = trans[0]
    pose_goal.position.y = trans[1]
    pose_goal.position.z = trans[2]

    poseStamped = PoseStamped()
    poseStamped.pose = pose_goal

    joint_angles = limb.joint_ordered_angles()
    waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
    waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

    
    traj.append_waypoint(waypoint.to_msg())

    if(counter >= 30):
        pub.publish(traj.to_msg())
        traj.clear_waypoints()
        counter = 0

    rate.sleep()
