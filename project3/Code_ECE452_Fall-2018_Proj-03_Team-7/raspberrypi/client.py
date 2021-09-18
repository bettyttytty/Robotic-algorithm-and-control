from __future__ import print_function

import grpc

import aruco_pb2
import aruco_pb2_grpc
import cv2
import numpy as np

server_ip = '10.107.208.86'

channel = grpc.insecure_channel('{}:50051'.format(server_ip))
stub = aruco_pb2_grpc.GreeterStub(channel)


def check_connect():
    response = stub.SayHello(aruco_pb2.Empty())
    assert response is not None


def coordinate():
    c = stub.Ask(aruco_pb2.Empty())
    rvec = [c.r.x, c.r.y, c.r.z]
    car_front, _ = cv2.Rodrigues(np.array(rvec))
    tvec = [c.t.x, c.t.y, c.t.z]
    return np.array(tvec), car_front[:, 0]


def goal_location():
    c = stub.Goal(aruco_pb2.Empty())
    tvec = [c.t.x, c.t.y, c.t.z]
    return np.array(tvec)
