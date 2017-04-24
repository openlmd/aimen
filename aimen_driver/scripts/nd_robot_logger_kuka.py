#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from kuka.logger_robot import LoggerRobot
import socket


class NdRobotLogger():
    def __init__(self):
        rospy.init_node('robot_state', anonymous=False)

        self.pub = rospy.Publisher(
            'joint_states', JointState, queue_size=10)

        self.msg_joint_state = JointState()
        self.msg_joint_state.name = ['joint_1', 'joint_2', 'joint_3',
                                     'joint_4', 'joint_5', 'joint_6']
        self.msg_joint_state.position = [0, 0, 0, 0, 0, 0]

        robot_ip = rospy.get_param('~robot_ip', '172.31.1.100')

        self.logger_robot = LoggerRobot()
        #self.logger_robot.connect(robot_ip)
        connected = False
        for i in range(10):
            try:
                self.logger_robot.connect(robot_ip)
                connected = True
                break
            except socket.timeout:
                print 'Timed out'
                rospy.logerr('Timed out')
                rospy.sleep(1)
                continue
            except socket.error:
                print 'Network is unreachable'
                rospy.logerr('Network is unreachable')
                rospy.sleep(1)
                continue
        if not connected:
            rospy.signal_shutdown('Socket timed out')
            rospy.spin()
            return
        self.talker()

    def talker(self):
        while not rospy.is_shutdown():
            if not self.logger_robot.read_raw_logger():
                break
            if len(self.logger_robot.float_joints) > 0:
                self.msg_joint_state.header.stamp = rospy.Time.now()
                joints_pose = self.logger_robot.float_joints.popleft()
                for i in range(0, len(joints_pose)): #len(joints_pose)-3
                    self.msg_joint_state.position[i] = (
                        np.deg2rad(joints_pose[i]))
                self.pub.publish(self.msg_joint_state)
            rospy.sleep(0.01)
        self.logger_robot.disconnect()


if __name__ == '__main__':
    try:
        NdRobotLogger()
    except rospy.ROSInterruptException:
        pass
