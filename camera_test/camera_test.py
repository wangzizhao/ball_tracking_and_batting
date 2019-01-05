import cv2
import time
import numpy as np

# For camera 1
video = cv2.VideoCapture('http://@127.0.0.1:8080/?action=stream/frame.mjpg')
# For camera 2
# video = cv2.VideoCapture('http://@127.0.0.1:8090/?action=stream/frame.mjpg')

print "fps:", video.get(cv2.CAP_PROP_FPS)
print "height:", video.get(cv2.CAP_PROP_FRAME_HEIGHT)
print "width:", video.get(cv2.CAP_PROP_FRAME_WIDTH)

# print the frame rate
print_time = True
time_cnt = 0
fps_list = []
last_time = 0.0

cnt = 0

while (video.isOpened()):
    cnt += 1
    if print_time:
        time_cnt += 1
        if time_cnt == 100:
            time_cnt = 0
            fps_list = []

    
    return_value, frame = video.read()
    # cv2.imshow('block_frame', frame)
    # cv2.imshow('block_frame', block_frame)

    if print_time:
        now = time.time()
        fps_list.append(1/(now - last_time))
        last_time = now
        print 'fps approximation: ', np.mean(fps_list)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([10,180,120])
    upper = np.array([23,255,255])
    block_frame = cv2.inRange(hsv, lower, upper)

    # Testing houghcircle
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray_frame = cv2.GaussianBlur(gray, (9,9), 0)
    # circles = cv2.HoughCircles(gray_frame, cv2.HOUGH_GRADIENT,1,20,
    # 	param1=50,param2=30,minRadius=0,maxRadius=0)
    # circles_draw = np.uint16(np.around(circles))
    # for i in circles[0,:]:
    # 	cv2.circle(gray,(i[0],i[1]),i[2],(0,255,0),2)
    # 	cv2.circle(gray,(i[0],i[1]),2,(0,0,255),3)

    # cv2.imshow('detected circles', gray)
    #


    KERNELS = [
        [5, 5],                # kernel for erdoe
        [3, 3],                # kernel for open
        [3, 3]                 # kernel for close
    ]

    erode_kernel = np.ones(KERNELS[0], np.uint8)
    block_frame = cv2.erode(block_frame, erode_kernel, iterations = 1)
    open_kernel = np.ones(KERNELS[1], np.uint8)
    block_frame = cv2.morphologyEx(block_frame, cv2.MORPH_OPEN, open_kernel)
    close_kernel = np.ones(KERNELS[2], np.uint8)
    block_frame = cv2.morphologyEx(block_frame, cv2.MORPH_CLOSE, close_kernel)

    new_im, contours, hierarchy = cv2.findContours(block_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        block_area = cv2.contourArea(cnt)
        # print "block_area = ", block_area
        if block_area > 200:
            cv2.drawContours(block_frame,cnt,-1,(255,255,0),3)

    cv2.imshow('block_frame', block_frame)

    # cv2.imshow('block_frame', block_frame)

    # num_frame = 1
    # time_end = time.time()

    # print "speed: ", (time_end - time_start) / num_frame

    
    # cv2.imshow('camera', image)

    # cv2.imwrite(str(cnt)+'.jpg', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()