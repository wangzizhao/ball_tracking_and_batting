import numpy as np
import cv2
import os
import time


LOWER = np.uint8([14,120,140])
UPPER = np.uint8([20,190,245])

LOWER = np.uint8([14,120,140])
UPPER = np.uint8([20,200,245])

KERNELS = [
        [5, 5],                # kernel for erdoe
        [3, 3],                # kernel for open
        [3, 3]                 # kernel for close
    ]

cap = cv2.VideoCapture('output2.avi')

while(cap.isOpened()):
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    block_frame = cv2.inRange(hsv, LOWER, UPPER)

    # erode_kernel = np.ones(KERNELS[0], np.uint8)
 #    block_frame = cv2.erode(block_frame, erode_kernel, iterations = 1)
    open_kernel = np.ones(KERNELS[1], np.uint8)
    block_frame = cv2.morphologyEx(block_frame, cv2.MORPH_OPEN, open_kernel)
    close_kernel = np.ones(KERNELS[2], np.uint8)
    block_frame = cv2.morphologyEx(block_frame, cv2.MORPH_CLOSE, close_kernel)

    new_frame = frame
    new_im, contours, hierarchy = cv2.findContours(block_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    # if len(contours) == 0:
    #     print "Not detected"

    for cnt in contours:
        block_area = cv2.contourArea(cnt)
        ##===============##===============##===============##
        ## test code for circular object detection
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        # print "block_area =================== ", block_area
        # block_area range filter
        if block_area > 0 and len(approx) > -1:
        ##===============##===============##===============##
        # if block_area > 10:
            (cx,cy),radius = cv2.minEnclosingCircle(cnt)
            cv2.circle(new_frame,(int(cx),int(cy)), int(10), (0,255,0), -1)   # # this is the test code to test the lcm information

    # print hsv()
    # for i in range(len(detect_cx)):
    #   print hsv[detect_cx[i]][detect_cy[i]]
    # print "=================="
    # print hsv

    cv2.imshow("new_frame", new_frame)

    # k = cv2.waitKey(5) & 0xFF
    # if k == 27:
    #   break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()