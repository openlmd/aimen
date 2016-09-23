#!/usr/bin/env python
import json
import rospy

from aimen_driver.srv import SrvRobotCommand
from aimen_driver.srv import SrvRobotCommandResponse

from abb.server_robot import ServerRobot


class NdRobotServer():
    def __init__(self):
        rospy.init_node('robot_service_server', anonymous=False)

        self.service = rospy.Service(
            'robot_send_command', SrvRobotCommand, self.cb_robot_command)

        robot_ip = rospy.get_param('~robot_ip', '192.168.30.4')
        self.server_robot = ServerRobot()
        self.server_robot.connect(robot_ip)

        rospy.on_shutdown(self.server_robot.disconnect)
        rospy.spin()

    def cb_robot_command(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.command)
        try:
            command = json.loads(data.command.lower())
            if 'get_pose' in command:
                pose_rob = self.process_command(command)
                return SrvRobotCommandResponse(str(pose_rob))
            response = self.process_command(command)
            return SrvRobotCommandResponse(response)
        except ValueError, e:
            print "Command is not json", e
            return SrvRobotCommandResponse("NOK")

    def process_command(self, command):
        for cmd in sorted(command, reverse=True):
            if cmd == 'tool':
                return self.server_robot.set_tool(command[cmd])
            elif cmd == 'workobject':
                return self.server_robot.workobject(command[cmd])
            elif cmd == 'speed':
                return self.server_robot.speed(command[cmd])
            elif cmd == 'pose':
                return self.server_robot.buffer_pose(command[cmd])
            elif cmd == 'move':
                return self.server_robot.move(command[cmd])
            elif cmd == 'movej':
                return self.server_robot.move(command[cmd], movel=False)
            elif cmd == 'move_ext':
                return self.server_robot.move_ext(command[cmd])
            elif cmd == 'path_move':
                if self.server_robot.buffer_len() > 0:
                    self.server_robot.buffer_execute()
            elif cmd == 'path_clear':
                self.server_robot.clear_buffer()
            elif cmd == 'get_pose':
                return self.server_robot.get_cartesian()
            elif cmd == 'wait':
                return self.server_robot.wait_time(command[cmd])
            elif cmd == 'program':
                return self.server_robot.set_group((command[cmd], 0))  # laser program 11
            elif cmd == 'laser':
                return self.server_robot.laser_ready(command[cmd])
            elif cmd == 'power':
                return self.server_robot.laser_power(command[cmd])
            elif cmd == 'powder':
                return self.server_robot.powder(command[cmd])
            elif cmd == 'carrier':
                return self.server_robot.massflow(command[cmd])
            elif cmd == 'turntable':
                return self.server_robot.disk(command[cmd])
            elif cmd == 'stirrer':
                stirrer = int((command[cmd] * 100) / 100)
                return '0 0'
            elif cmd == 'cancel':
                return self.server_robot.cancel_motion()
            elif cmd == 'reset_laser':
                return self.server_robot.reset_laser()
            elif cmd == 'reset_powder':
                return self.server_robot.reset_powder()
            else:
                return 'ERR_COMMAND'
                print 'Unknown command:', cmd


if __name__ == '__main__':
    try:
        NdRobotServer()
    except rospy.ROSInterruptException:
        pass
