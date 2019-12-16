#!/usr/bin/env python
# coding: utf-8

import rospy
import PyKDL as kdl
import rosnode
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import actionlib
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

class GripperClient(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)

    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1 ):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()

class Pose_pub:
    def __init__(self):
        self.arm_pub = rospy.Publisher("/crane_x7/arm_controller/command", JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher("/crane_x7/gripper_controller/command", JointTrajectory, queue_size=1)
        self.rate = rospy.Rate(10)
        self.gc = GripperClient()

    def update_pose(self):
        jt_arm = JointTrajectory()
        jt_arm.joint_names = rospy.get_param("/crane_x7/arm_controller/joints")

        jt_gripper = JointTrajectory()
        jt_gripper.joint_names = rospy.get_param("/crane_x7/gripper_controller/joint")

        rospy.sleep(0.5)

        jt_arm_tra = np.vstack((np.hstack((np.linspace(0.0, 0.0, 100), np.linspace(0.0, 0.0, 100))),
                            np.hstack((np.linspace(0.0, 1.0, 100), np.linspace(1.0, 0.0, 100))),
                            np.hstack((np.linspace(0.0, 0.0, 100), np.linspace(0.0, 0.0, 100))),
                            np.hstack((np.linspace(0.0, -2.0, 100), np.linspace(-2.0, 0.0, 100))),
                            np.hstack((np.linspace(0.0, 0.0, 100), np.linspace(0.0, 0.0, 100))),
                            np.hstack((np.linspace(0.0, 1.0, 100), np.linspace(1.0, 0.0, 100))),
                            np.hstack((np.linspace(0.0, 1.0, 100), np.linspace(1.0, 0.0, 100)))))

        jt_gripper_tra = np.hstack((np.linspace(0.0, 1.0, 100), np.linspace(1.0, 0.0, 100)))

        jt_arm_tra = jt_arm_tra.T
        jt_gripper_tra = jt_gripper_tra.T

        while not rospy.is_shutdown():
            for step in range(len(jt_arm_tra)):
                p_arm = JointTrajectoryPoint()

                for q in jt_arm_tra[step]:
                    p_arm.positions.append(q)

                p_arm.time_from_start = rospy.Duration(0.1)
                jt_arm.points = [p_arm]
                jt_arm.header.stamp = rospy.Time.now()
                self.arm_pub.publish(jt_arm)

                self.gc.command(jt_gripper_tra[step],1.0)

                self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('vr_controller')
        pose_pub = Pose_pub()
        pose_pub.update_pose()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
