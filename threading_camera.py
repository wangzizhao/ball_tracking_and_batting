import os
import shutil 
import threading
import json
import math
from operator import itemgetter, attrgetter

import time
import cv2
import numpy as np
import lcm
# from lcmtypes import camera_coord_t
import sys
sys.path.append("lcmtypes")
from lcmtypes.camera_coord_t import camera_coord_t
from camera import Camera_detector
from turntable import Turntable
from coord_calculator import Coord_calculator

# =============================================
#               Constants                     #
# =============================================
KERNELS = [
        [5, 5],                # kernel for erdoe
        [3, 3],                # kernel for open
        [3, 3]                 # kernel for close
    ]

# LOWER = np.array([10,180,120])
# UPPER = np.array([23,255,255])
LOWER = np.array([10,150,120])
UPPER = np.array([20,255,255])


class Camera_tracker(object):
    """docstring for Camera_tracker"""
    def __init__(self, camera_idxs):
        super(Camera_tracker, self).__init__()
        self.msg_1 = camera_coord_t()
        self.msg_2 = camera_coord_t()

        self.cam1 = Camera_detector(1)
        self.cam2 = Camera_detector(2)

        self.M1 = []
        self.M2 = []

        self.turntable1 = Turntable(1)
        self.turntable2 = Turntable(2)

        self.coord_calculator = Coord_calculator()


        self.threads = []
        t1 = threading.Thread(target = self.cam1.ball_detector)
        t1.start()
        self.threads.append(t1)

        t2 = threading.Thread(target = self.cam2.ball_detector)
        t2.start()
        self.threads.append(t2)

        t3 = threading.Thread(target = self.turntable1.rexarm_FK_loop)
        t3.start()
        self.threads.append(t3)

        t4 = threading.Thread(target = self.turntable2.rexarm_FK_loop)
        t4.start()
        self.threads.append(t4)


        while True:
            try:
                self.msg_1 = self.cam1.get_msg()
                self.msg_2 = self.cam2.get_msg()
                self.M1 = self.turntable1.get_M()
                self.M2 = self.turntable2.get_M()
                # print "M1: ", self.M1
                # print "M2: ", self.M2
                print "msg1: {}, {}, {}".format(self.msg_1.x, self.msg_1.y, self.msg_1.radius)
                # print "msg2: {}, {}, {}".format(self.msg_2.x, self.msg_2.y, self.msg_2.radius)
                base1 = self.calc_coord()
            except SystemExit:
                for thread in threads:
                    thread.join()
                return
            # print "base1: ", base1
            # if self.msg_1.radius > 0:
            #     img = np.zeros((512,512,3), np.uint8)
            #     print "msg1: {}, {}, {}".format(self.msg_1.x, self.msg_1.y, self.msg_1.radius)
            #     cv2.circle(img,(int(self.msg_1.x),int(self.msg_1.y)), int(min(self.msg_1.radius, 10)), (0,0,255), -1)
            #     cv2.imshow("img", img)
        # self.turntable1 = Turntable(1)
        # self.turnable2 = Turntable(2)
        # self.video1 = None
        # self.video2 = None

        # for idx in camera_idxs:
        #     idx = int(idx)
        #     if idx == 1:
        #         print "1 camera"
        #         self.video1 = cv2.VideoCapture('http://@127.0.0.1:8080/?action=stream/frame.mjpg')
        #         t1 = threading.Thread(target = Camera_tracker.ball_detector1, args = (self, 1, ))
        #         t1.start()
        #     elif idx == 2:
        #         print "2 camera"
        #         self.video2 = cv2.VideoCapture('http://@127.0.0.1:8090/?action=stream/frame.mjpg')
        #         t2 = threading.Thread(target = Camera_tracker.ball_detector2, args = (self, 2, ))
        #         t2.start()
        #     else:
        #         print "haha, wrong arg"


    def calc_coord(self):
        if len(self.M2) == 0 or len(self.M1) == 0:
            return -1
        Rb, Tb = self.coord_calculator.calc_RT_matrix(self.M1, self.M2)
        T1, T2 = self.coord_calculator.calc_pos_in_camera_coord(Rb, Tb, self.msg_1.x, self.msg_1.y, self.msg_2.x, self.msg_2.y)
        # print T1, T2
        base1 = self.coord_calculator.trans_camera_to_base(T1, self.    M1)
        return base1
        # base2 = trans_camera_to_base(T2, M2)
        # print "base1: ", base1


if __name__ == '__main__':
    print ((sys.argv[1:]))
    print "start"
    Camera_tracker((sys.argv[1:]))
