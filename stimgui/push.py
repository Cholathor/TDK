import time
import zmq

context = zmq.Context()
zmq_socket = context.socket(zmq.PUSH)
zmq_socket.bind("tcp://*:5557")
# Start your result manager and workers before you start your producers
for i in range(100):
    zmq_socket.send(b"HALLO")
