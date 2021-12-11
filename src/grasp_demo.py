#! /usr/bin/env python
import sys
from typing import Mapping
import rospy
from std_msgs.msg import Float64
import moveit_commander
from math import pi, tau, dist, fabs, cos
import geometry_msgs.msg


def main():
    moveit_commander.roscpp_initialize(['joint_states:=/panda/joint_states'])
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("panda_arm")
    #hand_group = moveit_commander.MoveGroupCommander("panda_hand")

    # Put the arm in the start position
    # We get the joint values from the group and change some of the values:
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0.04
    joint_goal[2] = -0.23
    joint_goal[3] = -0.10
    joint_goal[4] = 0.46
    joint_goal[5] = 0.62  # 1/6 of a turn
    joint_goal[6] = 0.81

    arm_group.go(joint_goal, wait=True)
    arm_group.stop()

    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.0206
    pose_goal.position.y = 0.0405
    pose_goal.position.z = 0.8913

    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)
    arm_group.execute(plan, wait=True)

    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
    arm_group.clear_pose_targets()

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#arm_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
# arm_group.stop()


# Open the gripper
#joint_hand_goal = hand_group.get_current_joint_values()
#joint_hand_goal[0] = -tau / 8
# joint_hand_goal[1] = -tau / 8
# hand_group.go(joint_hand_goal, wait=True)
# hand_group.stop()
pub1 = rospy.Publisher(
    '/panda/panda_finger1_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher(
    '/panda/panda_finger2_controller/command', Float64, queue_size=10)

# put the arm at the 1st grasping position
# pose_target = geometry_msgs.msg.Pose()
# pose_target.orientation.w = 0.6279607961703095
# pose_target.orientation.x = 0.47538032306559025
# pose_target.orientation.y = 0.5792109525094049
# pose_target.orientation.z = -0.21022240462984418
# pose_target.position.x = 0.16255830804038524
# pose_target.position.y = 0.06158456819069608
# pose_target.position.z = 0.9095288838235128
# arm_group.set_pose_target(pose_target)
#plan1 = arm_group.plan()
#plan1 = arm_group.go()
# pub1.publish(0.15)
# pub2.publish(0.15)

# put the arm at the 2nd grasping position
#pose_target.position.z = 0.6095288838235128
# arm_group.set_pose_target(pose_target)
#plan1 = arm_group.plan()
#plan1 = arm_group.go()

# pub1.publish(0.02)
# pub2.publish(0.02)

#pose_target.position.z = 0.950740076339809
# arm_group.set_pose_target(pose_target)
#plan1 = arm_group.plan()
#plan1 = arm_group.go()

# close the gripper
# joint_hand_goal[0] = 0
# joint_hand_goal[1] = 0
# hand_group.go(joint_hand_goal, wait=True)
# hand_group.stop()

# put the arm at the 3rd grasping position
# pose_target.position.z = 1.5
# arm_group.set_pose_target(pose_target)
# #plan1 = arm_group.plan()
# plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
