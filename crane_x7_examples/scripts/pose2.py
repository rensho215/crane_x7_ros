#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    print("vertical1")
    arm.set_named_target("vertical1")
    arm.go()

    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    print("vertical1")
    arm.set_named_target("vertical1")
    arm.go()
<<<<<<< HEAD

=======
    
>>>>>>> 769d33c6d513b8eaf37bf392da9470c770a5be40
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    print("home")
    arm.set_named_target("home")
    arm.go()

    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    print("vertical1")
    arm.set_named_target("vertical1")
    arm.go()

    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    print("vertical1")
    arm.set_named_target("vertical1")
    arm.go()

    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    print("home")
    arm.set_named_target("home")
    arm.go()
<<<<<<< HEAD


    # 手動で姿勢を指定するには以下のように指定
    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = 0.0
    target_pose.position.z = 0.624
    q = quaternion_from_euler( 0.0, 0.0, 0.0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )	# 目標ポーズ設定
    arm.go()							# 実行
    """

    # 移動後の手先ポーズを表示
=======
    
>>>>>>> 769d33c6d513b8eaf37bf392da9470c770a5be40
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
<<<<<<< HEAD
=======
                   
>>>>>>> 769d33c6d513b8eaf37bf392da9470c770a5be40
