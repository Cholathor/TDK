import serial
import time
import binascii
import sys
import typing
import numpy as np


class GripperControl:

    def __init__(self):
        self.gripper = None
        self.gripperConnected = False
        self.activated = False
        self.pos = 0
        self.speed = 100
        self.force = 100

    def activate(self):
        if self.gripperConnected:
            print("Activating gripper...")
            self.gripper.write(bytearray.fromhex("091003E80003060000000000007330"))  # clear rAct
            time.sleep(0.01)
            self.gripper.write(bytearray.fromhex("091003E800030601000000000072E1"))  # activate
            time.sleep(0.01)
            data = self.gripper.readline()  # read entire line to clear all data
            return True
        return False


    def readPosCurrent(self):
        self.gripper.write(bytearray.fromhex("090307D00003040E"))
        data = self.gripper.read(11)
        pos = data[7]
        current = data[8]

        return [pos, current]


    def setPos(self, pos, speed, force):

        speed = round(speed)
        force = round(force)

        pos = int(np.clip(pos, 0, 255))
        speed = int(np.clip(speed, 0, 255))
        force = int(np.clip(force, 0, 255))

        self.pos = pos
        self.speed = speed
        self.force = force

        command = "091003E8000306090000" + hex(pos)[2:].zfill(2) + hex(speed)[2:].zfill(2) + hex(force)[2:].zfill(2)
        # Generate crc and add to the command message
        crc = self.crc_comp(command)
        command = command + crc[2] + crc[3] + crc[0] + crc[1]
        self.gripper.write(bytearray.fromhex(command))
        data = self.gripper.read(8)


    def crc_comp(self, data_str):
        # Generate cyclic redundancy check for modbus RTU protocol
        # Inputs:
        # data_str: data message as a string (in hex)
        # Outputs:
        # crc: cyclic redundancy check as a string (in hex)
        # NOTE: message must be re-arranged so that lowest byte is sent first
        data = bytearray.fromhex(data_str)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if ((crc & 1) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return ("%04X" % (crc))

    def connect(self, comm_default=5):
        if len(sys.argv) > 1:
            if sys.argv[1].isdigit():
                print("Comm port specified: ", str(sys.argv[1]))
                comm_port = sys.argv[1]
            else:
                print("Default comm port used: ", str(comm_default))
                comm_port = comm_default
        else:
            print("Default comm port used: ", str(comm_default))
            comm_port = comm_default

        print("Connecting to gripper...")
        try:
            self.gripper = serial.Serial(port='COM' + str(comm_port),
                                    baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1)
            self.gripperConnected = True
        except:
            print("Error connecting to gripper.")
            self.gripperConnected = False

        if self.gripperConnected:
            self.activated = self.activate()