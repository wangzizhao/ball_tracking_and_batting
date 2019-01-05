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
LOWER = np.array([10,150,120])
UPPER = np.array([20,255,255])

if len(sys.argv) == 1:
    print("Explicitly provide the camera number please")
    sys.exit()

camera_idx = int(sys.argv[1])

# For camera 1
if camera_idx == 1:
    video = cv2.VideoCapture('http://@127.0.0.1:8080/?action=stream/frame.mjpg')
# For camera 2
elif camera_idx == 2:
    video = cv2.VideoCapture('http://@127.0.0.1:8090/?action=stream/frame.mjpg')

print "fps:", video.get(cv2.CAP_PROP_FPS)
print "height:", video.get(cv2.CAP_PROP_FRAME_HEIGHT)
print "width:", video.get(cv2.CAP_PROP_FRAME_WIDTH)

# print the frame rate
print_time = False
time_cnt = 0
fps_list = []
last_time = 0.0
cnt = 0 
lc = lcm.LCM()

while (video.isOpened()):
    cnt += 1
    if print_time:
        time_cnt += 1
        if time_cnt == 100:
            time_cnt = 0
            fps_list = []
    
    return_value, frame = video.read()

    # calculate the fps
    if print_time:
        now = time.time()
        fps_list.append(1/(now - last_time))
        last_time = now
        print 'fps approximation: ', np.mean(fps_list)

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

    new_im, contours, hierarchy = cv2.findContours(block_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    
    new_frame = frame # this is the test code to test the lcm information
    for cnt in contours:
        block_area = cv2.contourArea(cnt)
        # print "block_area =================== ", block_area
        # block_area range filter
        if block_area > 10:
            cv2.drawContours(block_frame,cnt,-1,(255,255,0),3)
            (cx,cy),radius = cv2.minEnclosingCircle(cnt)
            # cx = x
            # cy = y
            msg = camera_coord_t()
            msg.x = cx
            msg.y = cy
            msg.radius = radius
            cv2.circle(new_frame,(int(cx),int(cy)), int(radius), (0,0,255), -1)   # # this is the test code to test the lcm information
            if camera_idx == 1:
                print "channel,x,y,radius: ({}, {:.2f}, {:.2f}, {:.2f})".format(1, cx, cy, radius)
                lc.publish("CAMERA_COORD1",msg.encode())
            elif camera_idx == 2:
                print "channel,x,y,radius: ({}, {:.2f}, {:.2f}, {:.2f})".format(2, cx, cy, radius)
                lc.publish("CAMERA_COORD2",msg.encode())

    # cv2.imshow('block_frame', block_frame)
    cv2.imshow('new_frame', new_frame) # this is the test code to test the lcm information

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()