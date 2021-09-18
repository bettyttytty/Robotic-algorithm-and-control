from concurrent import futures
import time
import grpc
import aruco_pb2
import aruco_pb2_grpc
import json
import numpy as np
import cv2
import datetime

c = aruco_pb2.Coordinate()
g = aruco_pb2.Coordinate()

aruco = cv2.aruco
aruco_dict = aruco.Dictionary_get(14)
charuco_board = aruco.CharucoBoard_create(7, 5, 0.04, 0.02, aruco_dict)


with open('calibrate.txt', 'r') as f:
    d = json.loads(f.read())
    cm = np.array(d['cm'])
    dc = np.array(d['dc'])


class Greeter(aruco_pb2_grpc.GreeterServicer):
    def SayHello(self, request, context):
        return aruco_pb2.HelloReply(t=int(time.time()))

    def Ask(self, request, context):
        return c

    def Goal(self, request, context):
        return g


server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
aruco_pb2_grpc.add_GreeterServicer_to_server(Greeter(), server)
server.add_insecure_port('[::]:50051')
server.start()
print "starting server"

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters,
                                          cameraMatrix=cm, distCoeff=dc)
    if ids is None:
        tmp = frame
    else:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.02, cm, dc)
        tmp = aruco.drawDetectedMarkers(frame, corners, ids)

        for i, j in enumerate(ids):
            if j == 0:
                c.r.x, c.r.y, c.r.z = rvec[i].flatten()
                c.t.x, c.t.y, c.t.z = tvec[i].flatten()
                c.st = time.time()

            if j == 3:
                g.r.x, g.r.y, g.r.z = rvec[i].flatten()
                g.t.x, g.t.y, g.t.z = tvec[i].flatten()
                g.st = time.time()

            tmp = aruco.drawAxis(tmp, cm, dc, rvec[i], tvec[i], 0.08)

    tmp = cv2.resize(tmp, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow('result', tmp)

    k = cv2.waitKey(1) & 0xFF

    if k == ord('q'):
        break
    elif k == ord('i'):
        print "-" * 20
        print "Current time is:", str(datetime.datetime.now())
        dt = datetime.datetime.fromtimestamp(c.st)
        print "Bot last update time:", str(dt)
        dt = datetime.datetime.fromtimestamp(g.st)
        print "Goal last update time:", str(dt)
        print "-" * 20

cap.release()
server.stop(0)
