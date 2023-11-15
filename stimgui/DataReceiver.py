import zmq
import time
import numpy as np
import threading
from GripperControl import GripperControl

class DataReceiver:

    connected = False
    socket = None

    def isConnected(self):
        return self.connected

    def openConnection(self, port: int, gripper):
        print("Opening connection...")
        t = threading.Thread(target=self.p_openConnection, args=[port, gripper])
        t.start()

    def p_openConnection(self, port: int, gripper):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{port}")

        while True:
            #  Wait for next request from client
            message = self.socket.recv()
            if message == b"CONNECTIONREQUEST":
                print("Received request: %s" % message)
                print("Connected successfully")
                self.connected = True
                self.socket.send(b"ACK")
                break
            else:
                print("Wrong request: %s" % len(message))
                self.connected = False
                self.socket.send(b"NACK")

            time.sleep(0.1)

        self.socket.close()
        self.socket = context.socket(zmq.PULL)
        self.socket.connect("tcp://localhost:5555")

        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)

        while True:
            sock = dict(poller.poll())
            if self.socket in sock and sock[self.socket] == zmq.POLLIN:
                message = self.socket.recv()

                message = str(message)[2:-1]
                message = [int(np.clip(int(i), -200, 200)) for i in message.split(',')[:-1]]

                value = np.mean(message)
                print(value)
                if value >= 100:
                    gripper.setPos(255, 1, 100)
                elif value >= 50:
                    gripper.setPos(gripper.readPosCurrent()[0], 1, 100)
                else:
                    gripper.setPos(0, 1, 100)
            time.sleep(0.01)


    def startReceiving(self, gripper: GripperControl):
        print("Starting gripper control...")
        t = threading.Thread(target=self.p_startReceiving, args=[gripper])


    def p_startReceiving(self, gripper: GripperControl):
        while True:
            message = self.socket.recv()

            value = int(np.clip(int(message), -127, 127)) + 127
            print(value)
            if value >= 170:
                gripper.setPos(255, 1, 100)
            elif value >= 85:
                gripper.setPos(gripper.readPosCurrent()[0], 1, 100)
            else:
                gripper.setPos(0, 1, 100)
            self.socket.send(b"A")