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
from tf.transformations import quaternion_from_matrix
from intera_interface import Limb
import numpy as np

from leap_motion.msg import leap

rospy.init_node('sawyer_collect_from_kinect', anonymous=True)

limb = Limb()

wpt_opts = MotionWaypointOptions(max_linear_speed=20.0,
                                    max_linear_accel=10.0,
                                    max_rotational_speed=20.0,
                                    max_rotational_accel=10.0,
                                    max_joint_speed_ratio=1.0,
                                    corner_distance=0.00)


traj_options = TrajectoryOptions()
traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
# traj_options.path_interpolation_step = 0.05

orientation = Quaternion()

def update_hand(msg):
    global orientation
    try:
        orientation = Quaternion(matrix=np.array(msg.data).reshape((3, 3)))
    except ValueError as e:
        rospy.logerr("Special orthogonal")
        rospy.logerr(np.array(msg.data).reshape((3, 3)))
        orientation = Quaternion(matrix=-1*np.array(msg.data).reshape((3, 3)))

def update_hand_ypr(msg):
    global orientation
    rospy.logwarn(msg.ypr)
    orientation = quaternion_from_euler(msg.ypr.x, msg.ypr.y, msg.ypr.z)
        

listener = tf.TransformListener()
# hand = rospy.Subscriber('/leapmotion/arm', Float64MultiArray, update_hand)
# ypr_sub = rospy.Subscriber('/leapmotion/data', leap, update_hand_ypr)
pub = rospy.Publisher('ssg_trajectory', Trajectory, queue_size=10)

BASE_X = 0.5
BASE_Y = 0.0
BASE_Z = 0.3
# OR_X = 0.61864
# OR_Y = -0.1366
# OR_Z =  0.77097
# OR_W = -0.065021
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
        (trans, _) = listener.lookupTransform('kinect/user_1/torso', 'kinect/user_1/left_hand', rospy.Time(0))
        (arm_dir, _) = listener.lookupTransform('kinect/user_1/left_elbow', 'kinect/user_1/left_hand', rospy.Time(0))
        # (shoulder_dir, _) = listener.lookupTransform('kinect/user_1/left_shoulder', 'kinect/user_1/left_elbow', rospy.Time(0))
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print 'excepted exception ' + str(type(e))
        trans = [0.0,0.0,0.0]
        arm_dir = [1,0,0]
    shoulder_dir = [1,1,0]

    counter += 1
    rospy.loginfo( "setting goal state: " + str(trans))

    pose_goal = Pose()

    arm_dir = np.array([-arm_dir[2], -arm_dir[0], arm_dir[1]])

    D = arm_dir / np.linalg.norm(arm_dir)
    S = np.cross(D, np.array(shoulder_dir))
    S = S / np.linalg.norm(S)
    U = np.cross(D, S) 
    U = U / np.linalg.norm(U)


    rot_mat = np.array([D,U,S])
    # rospy.logwarn(rot_mat)

    try:
        orientation = Quaternion(matrix=rot_mat)
    except ValueError:
        orientation = Quaternion(matrix=-rot_mat)

    # rospy.logwarn(str(orientation))

    # pose_goal.orientation.w = orientation.elements[0]
    # pose_goal.orientation.x = orientation.elements[1]
    # pose_goal.orientation.y = orientation.elements[2]
    # pose_goal.orientation.z = orientation.elements[3]

    # pose_goal.orientation.w = orientation[3]
    # pose_goal.orientation.x = orientation[0]
    # pose_goal.orientation.y = orientation[1]
    # pose_goal.orientation.z = orientation[2]

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
