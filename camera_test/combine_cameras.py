import cv2
import time
import numpy as np

# =============================================
#               Constants                     #
# =============================================
KERNELS = [
        [5, 5],                # kernel for erdoe
        [3, 3],                # kernel for open
        [3, 3]                 # kernel for close
    ]

# LOWER = np.array([10,180,120])
# upper = np.array([23,255,255])
LOWER = np.array([10,150,120])
UPPER = np.array([20,255,255])


# For camera 1
video1 = cv2.VideoCapture('http://@127.0.0.1:8080/?action=stream/frame.mjpg')
# For camera 2
video2 = cv2.VideoCapture('http://@127.0.0.1:8090/?action=stream/frame.mjpg')

def print_info(video):
    print "fps:", video.get(cv2.CAP_PROP_FPS)
    print "height:", video.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print "width:", video.get(cv2.CAP_PROP_FRAME_WIDTH)

# print the frame rate
print_time = True
time_cnt = 0
fps_list = []
last_time = 0.0

cnt = 0

def detectBlob(video, last_time, name):
    return_value, frame = video.read()

    cv2.imshow('orignal' + name, frame)

    # calculate the fps
    # if print_time:
    #     now = time.time()
    #     fps_list.append(1/(now - last_time))
    #     last_time = now
    #     print 'fps approximation: ', np.mean(fps_list)

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

    for cnt in contours:
        block_area = cv2.contourArea(cnt)
        # print "block_area =================== ", block_area
        if block_area > 10:
            cv2.drawContours(block_frame,cnt,-1,(255,255,0),3)
            # M = cv2.moments(cnt)
            # cx = int(M['m10']/M['m00'])
            # cy = int(M['m01']/M['m00'])
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            cx = int(x)
            cy = int(y)
            radius = int(radius)
            # print "x,y,radius: ({}, {}, {})".format(cx, cy, radius)

    cv2.imshow('block_frame' + name, block_frame)


while (video1.isOpened() and video2.isOpened()):
    cnt += 1
    if print_time:
        time_cnt += 1
        if time_cnt == 100:
            time_cnt = 0
            fps_list = []

    detectBlob(video1, 0, "1")
    detectBlob(video2, 0, "2")
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video1.release()
video2.release();
cv2.destroyAllWindows()