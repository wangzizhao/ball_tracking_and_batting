import cv2
import time
import numpy as np
# for the lcm communication for camera
import sys
import lcm
sys.path.append("../lcmtypes")
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
# LOWER = np.array([13,150,120])
# UPPER = np.array([20,255,255])
# LOWER = np.uint8([15,135,170])
# UPPER = np.uint8([19,180,230])

# for frame1
LOWER = np.uint8([15,135,170])
UPPER = np.uint8([19,180,245])
detect_cx = []
detect_cy = []
first_flag = True

# ==================
# [ 34 255  14]
# [ 28  99 162]
# [ 25 255  29]
# [ 60 255 255]
# [ 27  92 195]
# [ 28  95 199]
# [ 31 101 156]
# [ 27 102 175]
# [ 29  87 185]
# [ 27  93 191]
# [ 29  86 198]
# [ 31  76 189]
# ==================
# [ 18 104  66]


while True:
	# LOWER = np.array([15,160,120])
	# UPPER = np.array([19,180,255])

	frame = cv2.imread("frame2_camera.png")
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	block_frame = cv2.inRange(hsv, LOWER, UPPER)

	new_frame = frame
	new_im, contours, hierarchy = cv2.findContours(block_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

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
	    	print "detected"
	        (cx,cy),radius = cv2.minEnclosingCircle(cnt)
	    	cv2.circle(new_frame,(int(cx),int(cy)), int(radius), (0,0,255), -1)   # # this is the test code to test the lcm information
	    	if first_flag == True:
	    		detect_cx.append(cx)
	    		detect_cy.append(cy)

	first_flag = False
	# print hsv()
	# for i in range(len(detect_cx)):
	# 	print hsv[detect_cx[i]][detect_cy[i]]
	# print "=================="
	# print hsv

	cv2.imshow("new_frame", new_frame)

	# k = cv2.waitKey(5) & 0xFF
	# if k == 27:
	# 	break
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cv2.destroyAllWindows()