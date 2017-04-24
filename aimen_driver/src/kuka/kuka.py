'''
kuka.py: contains classes and support functions which interact with an KUKA Robot
 running our software stack (KRL code module SERVER)


For functions which require targets (XYZ positions with quaternion
orientation), targets can be passed as [[XYZ], [Quats]] OR [XYZ, Quats]

'''

import socket
from collections import deque
import logging
from xml.etree import ElementTree as ET
from struct import *

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


class Robot:
    def __init__(self):
        self.control = True
        self.delay = .08
        TCP_IP = '172.31.1.100'
        TCP_PORT = 59152

    def connect_logger(self, remote, maxlen=10):
        self.pose = deque(maxlen=maxlen)
        self.joints = deque(maxlen=maxlen)
        self.float_joints = deque(maxlen=maxlen)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.s.bind((TCP_IP,TCP_PORT))
        self.s.connect(remote)
        self.s.setblocking(1)

    def read_logger(self):
        '''Unpack data from xml file'''
        data = self.s.recv(4096).split()
        self.joints.append(data)

    def read_raw_logger(self):
        '''Unpack raw data from xml file'''
        try:
            raw_data = self.s.recv(4096)
            if len(raw_data) > 27:
                n_joints = unpack('i', raw_data[:4])[0]
                if n_joints == 32:
                    self.float_joints.append(unpack('ffffffff',
                                             raw_data[4:n_joints+4]))
                elif n_joints == 24:
                    self.float_joints.append(unpack('ffffff',
                                             raw_data[4:n_joints+4]))
            return True
        except socket.error, e:
            return False

    def read_xml_logger(self):
        '''
        Decode robot axis positions from a Kuka xml message
        '''
        try:
            data = self.s.recv(512)
            if data:
                data_XML = self.recibirXML(data)
                if len(data[0] == 6):
                    for axis in data_XML[0]:
                        self.float_joints.append(axis)
            return True
        except socket.error, e:
            return False

    def recibirXML(self, sender):
        '''
        Read data from robot to a XML structure
        @author: lucia.alonso
        '''
        rec_unicode = sender.decode('utf-8')
        rec_string = str(rec_unicode)
        # Convertir string en XML
        XML = ET.fromstring(rec_string)
        return XML
