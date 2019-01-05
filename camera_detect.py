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
## =====================================    ]
## the range used before, very terrible
# LOWER = np.array([10,180,120])
# UPPER = np.array([23,255,255])
# LOWER = np.array([13,150,120])
# UPPER = np.array([20,255,255])

## =====================================
## the range is okay for now
# LOWER = np.array([15,135,180])
# UPPER = np.array([19,180,230])
# LOWER = np.uint8([15,135,170])
# UPPER = np.uint8([19,180,230])


## =====================================
## the range is okay for good
# LOWER = np.uint8([13,250,180])
# UPPER = np.uint8([20,255,255])

# LOWER = np.uint8([13,240,180])
# UPPER = np.uint8([20,255,255])


DISTANCE_THRSHOLD = 50

## =============================================
#               Funtions                     #
# =============================================
# def calc_dist_square(prev_x, prev_y, cur_x, cur_y):
#     print "(prev_x - cur_x): ", (prev_x - cur_x)
#     print "(prev_y - cur_y): ", (prev_y - cur_y)
#     return (prev_x - cur_x) * (prev_x - cur_x) + (prev_y - cur_y) * (prev_y - cur_y)

# def check_history_distance(prev_msg, cur_msg):
#     if prev_msg.x == -1:
#         return True
#     print "distance: ", calc_dist_square(prev_msg.x, prev_msg.y, cur_msg.x, cur_msg.y)
#     return calc_dist_square(prev_msg.x, prev_msg.y, cur_msg.x, cur_msg.y) <= DISTANCE_THRSHOLD 

if len(sys.argv) == 1:
    print("Explicitly provide the camera number please")
    sys.exit()

camera_idx = int(sys.argv[1])

# if camera_idx == 1:
#     LOWER = np.uint8([14,120,140])
#     UPPER = np.uint8([20,180,245])
# elif camera_idx == 2:
#     LOWER = np.uint8([14,120,140])
#     UPPER = np.uint8([22,185,255])

# For camera 1
if camera_idx == 1:
    # LOWER = np.uint8([13,210,170])
    # UPPER = np.uint8([20,255,255])
    # LOWER = np.uint8([13,180,160])
    # UPPER = np.uint8([20,255,255])

    LOWER = np.uint8([8,140,100])
    UPPER = np.uint8([25,255,255])
    video = cv2.VideoCapture('http://@127.0.0.1:8080/?action=stream/frame.mjpg')
# For camera 2
elif camera_idx == 2:   
    # LOWER = np.uint8([13,210,170])
    # UPPER = np.uint8([21,255,255])
    LOWER = np.uint8([10,120,160])
    UPPER = np.uint8([21,255,255])
    video = cv2.VideoCapture('http://@127.0.0.1:8090/?action=stream/frame.mjpg')

print "fps:", video.get(cv2.CAP_PROP_FPS)
print "height:", video.get(cv2.CAP_PROP_FRAME_HEIGHT)
print "width:", video.get(cv2.CAP_PROP_FRAME_WIDTH)

# print the frame rate
print_time = True
time_cnt = 0
fps_list = []
last_time = 0.0
cnt = 0 
lc = lcm.LCM()

# previous msg
# prev_msg = camera_coord_t()
# prev_msg.x = -1
# prev_msg.y = -1
# prev_msg.z = -1

# save video
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output' +str(camera_idx) + '.avi',fourcc, 20.0, (640,480))

while (video.isOpened()):
    cnt += 1
    if print_time:
        time_cnt += 1
        if time_cnt == 100:
            time_cnt = 0
            fps_list = []
    
    return_value, frame = video.read()

    # test code for filter the frame
    # frame = cv2.GaussianBlur(frame, (5,5), 0)

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
    msg.z = -1
    msg.v_x = -1
    msg.v_y = -1
    msg.v_z = -1
    msg.utime = int(time.time()*1e6)
    msg.mode = -1
    temp_r = 0

    for cnt in contours:
        # block_area = cv2.contourArea(cnt)
        ##===============##===============##===============##
        ## test code for circular object detection
        # approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        # print "block_area =================== ", block_area
        # block_area range filter
        # if len(approx) > 2:
        ##===============##===============##===============##
        # if block_area > 10:
        (cx,cy),radius = cv2.minEnclosingCircle(cnt)

        if radius > temp_r:
            msg.x = cx
            msg.y = cy
            temp_r = radius
            cv2.drawContours(block_frame,cnt,-1,(255,255,0),3)
        cv2.circle(new_frame,(int(cx),int(cy)), int(radius), (255,255,0), -1)   # # this is the test code to test the lcm information

    # =============================================
    # # test code for the camera calibration      #
    # =============================================
    (x_range, y_range, channel) = new_frame.shape
    # cv2.circle(new_frame, (int(y_range / 2),int(x_range / 2)), int(10), (0,0,255), -1)
    cv2.line(new_frame, (int(y_range / 2), 0), (int(y_range / 2), x_range), (0,255,0), 1)
    cv2.line(new_frame, (0, int(x_range / 2)), (int(y_range), int(x_range / 2)), (0,255,0), 1)

    if camera_idx == 1:
        # print "channel,x,y,radius: ({}, {:.2f}, {:.2f}, {:.2f})".format(1, msg.x, msg.y, msg.radius)
        lc.publish("CAMERA_COORD1",msg.encode())
        # cv2.imshow('block_frame1', block_frame)
        cv2.imshow('new_frame1', new_frame) # this is the test code to test the lcm information
    elif camera_idx == 2:
        # print "channel,x,y,radius: ({}, {:.2f}, {:.2f}, {:.2f})".format(2, msg.x, msg.y, msg.radius)
        lc.publish("CAMERA_COORD2",msg.encode())
        # cv2.imshow('block_frame2', block_frame)
        cv2.imshow('new_frame2', new_frame) # this is the test code to test the lcm information
    # calculate the fps
    if print_time:
        now = time.time()
        fps_list.append(1/(now - last_time))
        last_time = now
        print 'fps approximation: ', np.mean(fps_list)

    # out.write(new_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# out.release()
video.release()
cv2.destroyAllWindows()