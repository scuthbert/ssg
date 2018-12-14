#!/usr/bin/env python

import roslib
roslib.load_manifest('ssg')
import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
import tf
from intera_interface import Limb


rospy.init_node('sawyer_moveit_from_topic_irl',
                anonymous=True)

limb = Limb()

wpt_opts = MotionWaypointOptions(max_linear_speed=0.4,
                                    max_linear_accel=0.4,
                                    max_rotational_speed=1.57,
                                    max_rotational_accel=1.57,
                                    max_joint_speed_ratio=1.0)


traj_options = TrajectoryOptions()
traj_options.interpolation_type = TrajectoryOptions.CARTESIAN

listener = tf.TransformListener()

rate = rospy.Rate(4.0) # SUGGEST - Try 10 or more
while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('kinect/user_1/torso', 'kinect/user_1/right_hand', rospy.Time(0))
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print 'excepted exception ' + str(type(e))
        rate.sleep()
        continue

    print "setting goal state: " + str(trans)

    pose_goal = Pose()
    pose_goal.orientation.w = 0.0
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 1.0
    pose_goal.orientation.z = 0.0

    trans = [0.2 * trans[0] + 0.6, -0.2 * trans[2] + 0.3, 0.2 * trans[1] + 0.2]

    pose_goal.position.x = trans[0]
    pose_goal.position.y = trans[1]
    pose_goal.position.z = trans[2]

    poseStamped = PoseStamped()
    poseStamped.pose = pose_goal


    traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

    joint_angles = limb.joint_ordered_angles()
    waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
    waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

    traj.append_waypoint(waypoint.to_msg())

    result = traj.send_trajectory(timeout=1.0)

    if result is None:
        rospy.logerr('Trajectory FAILED to send')
        rate.sleep()
    else:
        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s', result.errorId)


    rate.sleep()
