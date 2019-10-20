#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

import copy

def main():
    rospy.init_node("set_waypoint_with_path")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # SRDFに定義されている"home"の姿勢にする
    print("home")
    arm.set_named_target("home")
    arm.go()

    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    waypoints = []

    wpose = arm.get_current_pose().pose
    
    # 上方から掴む
    q = quaternion_from_euler(-3.14, 0.0, 0.0)  
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]
    waypoints.append(copy.deepcopy(wpose)) # Waypointの追加

    wpose.position.z -= 0.15
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z += 0.15
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 0.1
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z -= 0.15
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z += 0.15
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x -= 0.1
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Pathを生成する
    (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)

    # 0.1倍速で動かす
    plan = arm.retime_trajectory(robot.get_current_state(), plan, 0.1)

    # Planを実行
    arm.execute(plan)

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
