import numpy as np
import cv2
import os
import time

cap = cv2.VideoCapture('v1.MOV')

# # File name 
# FILE_OUTPUT = 'output.mp4'

# # Checks and deletes the output file
# # You cant have a existing file or it will through an error
# if os.path.isfile(FILE_OUTPUT):
#     os.remove(FILE_OUTPUT)

# # Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
# out = cv2.VideoWriter(FILE_OUTPUT,fourcc, 20.0, (640,480))

time_start = time.time()
num_frame = 0

while(cap.isOpened()):
    ret, frame = cap.read()
    time_start = time.time()
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('frame',frame)

    # im, contours, hierarchy = cv2.findContours(frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    # for cnt in contours:
    #     # block_area = cv2.contourArea(cnt)
    #     # print "block_area = ", block_area
    #     rect = cv2.minAreaRect(cnt)
    #     box = cv2.boxPoints(rect)
    #     box = np.int0(box)
    #     print "box: ", box
    #     cv2.drawContours(frame,[box],0,(0,0,255),2)

    # cv2.imshow('new',frame)    

    
    
    # =============================================
    #        SimpleBlobDetector Failed            #
    # =============================================
    # params = cv2.SimpleBlobDetector_Params()
    # params.filterByArea = True
    # params.maxArea = 1200
    # params.minArea = 400

    # params.filterByCircularity = True
    # params.minCircularity = 0.6
    # params.maxCircularity = 1.0

    # # params.filterByColor = True
    # # params.blobColor = 100

    # # Set up the detector with default parameters.
    # detector = cv2.SimpleBlobDetector_create(params)
     
    # # Detect blobs.
    # keypoints = detector.detect(frame)
     
    # # Draw detected blobs as red circles.
    # # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    # im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # =====  End of SimpleBlobDetector ====== #
    
    # =============================================
    #        REXARM LAB WAY Succeed!            #
    # =============================================
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([10,180,120])
    upper = np.array([23,255,255])
    block_frame = cv2.inRange(hsv, lower, upper)

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
        if block_area > 100:
            cv2.drawContours(block_frame,cnt,-1,(255,255,0),3)

    cv2.imshow('block_frame', block_frame)

    num_frame = 1
    time_end = time.time()

    print "speed: ", (time_end - time_start) / num_frame
    # # write the masked frame
    # out.write(block_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# out.release()
cap.release()
cv2.destroyAllWindows()