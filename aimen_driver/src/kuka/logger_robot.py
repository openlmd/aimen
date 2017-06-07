import socket
from kuka import Robot


class LoggerRobot(Robot):
    def __init__(self):
        Robot.__init__(self)

    def connect(self, ip):
        self.control = True
        self.connect_logger((ip, 59152))
        #TODO: Ler porto da configuracion

    def disconnect(self):
        self.conn.close()
        self.s.close()
        self.s.shutdown(socket.SHUT_RDWR)


if __name__ == '__main__':
    logger_robot = LoggerRobot()
    logger_robot.connect('172.31.1.100')
    logger_robot.disconnect()
