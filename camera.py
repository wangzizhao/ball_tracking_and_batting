import cv2
import time
import numpy as np
# for the lcm communication for camera
import sys
import lcm
sys.path.append("lcmtypes")
# from lcmtypes import camera_coord_t
from lcmtypes.camera_coord_t import camera_coord_t

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

class Camera_detector(object):
    """docstring for Camera_detector"""
    def __init__(self, camera_idx):
        # super(Camera_detector, self).__init__()
        self.camera_idx = camera_idx
        self.msg = camera_coord_t()
        # self.msg.x = -1
        # self.msg.y = -1
        # self.msg.radius = -1
        
        # For camera 1
        if self.camera_idx == 1:
            self.video = cv2.VideoCapture('http://@127.0.0.1:8080/?action=stream/frame.mjpg')
        # For camera 2
        elif self.camera_idx == 2:
            self.video = cv2.VideoCapture('http://@127.0.0.1:8090/?action=stream/frame.mjpg')


    def ball_detector(self):
        # print the frame rate
        print_time = True
        time_cnt = 0
        fps_list = []
        last_time = 0.0
        cnt = 0 

        while (self.video.isOpened()):
            cnt += 1
            if print_time:
                time_cnt += 1
                if time_cnt == 100:
                    time_cnt = 0
                    fps_list = []
            
            return_value, frame = self.video.read()

            # test code for filter the frame
            frame = cv2.GaussianBlur(frame, (5,5), 0)

            # =============================================
            #        REXARM LAB WAY Succeed!            #
            # =============================================
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            block_frame = cv2.inRange(hsv, LOWER, UPPER)

            # erode_kernel = np.ones(KERNELS[0], np.uint8)
            # block_frame = cv2.erode(block_frame, erode_kernel, iterations = 1)
            open_kernel = np.ones(KERNELS[1], np.uint8)
            block_frame = cv2.morphologyEx(block_frame, cv2.MORPH_OPEN, open_kernel)
            close_kernel = np.ones(KERNELS[2], np.uint8)
            block_frame = cv2.morphologyEx(block_frame, cv2.MORPH_CLOSE, close_kernel)

            # # test code for edge detection
            # block_frame = cv2.Canny(block_frame, 75, 200)
            # # cv2.imshow("after edge", block_frame)

            new_im, contours, hierarchy = cv2.findContours(block_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            
            new_frame = frame # this is the test code to test the lcm information
            msg = camera_coord_t()
            msg.x = -1
            msg.y = -1
            msg.radius = -1

            for cnt in contours:
                block_area = cv2.contourArea(cnt)
                ##===============##===============##===============##
                ## test code for circular object detection
                approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
                # print "block_area =================== ", block_area
                # block_area range filter
                if block_area > 10 and len(approx) > 10:
                ##===============##===============##===============##
                # if block_area > 10:
                    cv2.drawContours(block_frame,cnt,-1,(255,255,0),3)
                    (cx,cy),radius = cv2.minEnclosingCircle(cnt)
                    # cx = x
                    # cy = y
                    if radius > msg.radius:
                        msg.x = cx
                        msg.y = cy
                        msg.radius = radius
                    cv2.circle(new_frame,(int(cx),int(cy)), int(radius), (0,0,255), -1)   # # this is the test code to test the lcm information
            
            self.msg.x = msg.x
            self.msg.y = msg.y
            self.msg.radius = msg.radius

            # cv2.imshow('block_frame' + str(self.camera_idx), block_frame)
            # cv2.imshow('new_frame', new_frame) # this is the test code to test the lcm information
            # calculate the fps
            if print_time:
                now = time.time()
                fps_list.append(1/(now - last_time))
                last_time = now
                # print 'fps approximation: ', np.mean(fps_list)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.video.release()
        cv2.destroyAllWindows()

    def get_msg(self):
        return self.msg

    def get_camera_idx(self):
        return self.camera_idx