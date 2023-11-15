import time
import zmq
import numpy as np

context = zmq.Context()

socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

print("Opened connection")
while True:

    message = socket.recv()
    if message == b"CONNECTIONREQUEST":
        print("Received request: %s" % message)
        print("Connected successfully")
        connected = True
        socket.send(b"ACK")
        break
    else:
        print("Wrong request: %s" % len(message))
        connected = False
        socket.send(b"NACK")

    time.sleep(0.1)

socket.close()

socket = context.socket(zmq.PULL)
socket.connect("tcp://localhost:5555")

poller = zmq.Poller()
poller.register(socket, zmq.POLLIN)

print("hahah")


while True:
    sock = dict(poller.poll())
    if socket in sock and sock[socket] == zmq.POLLIN:
        message = socket.recv()
        message = str(message)[2:-1]
        message = [int(np.clip(int(i), -200, 200)) for i in message.split(',')[:-1]]

        value = np.mean(message)

        print(value)

