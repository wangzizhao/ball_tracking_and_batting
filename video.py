import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
import freenect

rgb_black   = np.uint8([[[50, 50, 50]]])
rgb_red     = np.uint8([[[133, 53, 70]]])
rgb_orange  = np.uint8([[[160, 90, 60]]])
rgb_yellow  = np.uint8([[[250, 250, 40]]])
rgb_green   = np.uint8([[[60, 170, 60]]])
rgb_blue    = np.uint8([[[65, 80, 160]]])
rgb_violet  = np.uint8([[[100, 80, 110]]])
rgb_pink    = np.uint8([[[200, 90, 120]]])
rgb_none    = np.uint8([[[0, 0, 0]]])

hsv_black   = cv2.cvtColor(rgb_black,cv2.COLOR_RGB2HSV)
hsv_red     = cv2.cvtColor(rgb_red,cv2.COLOR_RGB2HSV)
hsv_orange  = cv2.cvtColor(rgb_orange,cv2.COLOR_RGB2HSV)
hsv_yellow  = cv2.cvtColor(rgb_yellow,cv2.COLOR_RGB2HSV)
hsv_green   = cv2.cvtColor(rgb_green,cv2.COLOR_RGB2HSV)
hsv_blue    = cv2.cvtColor(rgb_blue,cv2.COLOR_RGB2HSV)
hsv_violet  = cv2.cvtColor(rgb_violet,cv2.COLOR_RGB2HSV)
hsv_pink    = cv2.cvtColor(rgb_pink,cv2.COLOR_RGB2HSV)

colors = \
{
    "black"     : rgb_black,
    "red"       : rgb_red,
    "orange"    : rgb_orange,
    "yellow"    : rgb_yellow,
    "green"     : rgb_green,
    "blue"      : rgb_blue,
    "violet"    : rgb_violet,
    # "pink"      : rgb_pink,
}

threshold = 15

class Video():
    def __init__(self):
        self.currentVideoFrame=np.array([])
        self.currentDepthFrame=np.array([])
        self.colorMaskFrame=np.array([])
        self.depthMaskFrame=np.array([])
        self.blockContours=[]
        self.blockCenters=[]
        self.kinectConnected = 1 #expect kinect to be available

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])


        """
        Calibration Variables

        Currently this takes three points to perform a simple calibration.
        The simple calibration only worls for points on the board and uses
        an affine transform to perform the calibration.

        To use:
            1. Click on simple calibration button
            2. Click the center of the arm
            3. Click the upper right corner of the board
            4. Click the lower right corner of the board

        Note that OpenCV requires float32 arrays

        TODO: Modify these to enable a robust 3D calibration

        """
        # TODO: measure board
        self.max_x = 300.;
        self.max_y = 300.;
        self.min_x = -self.max_x;
        self.min_y = -self.max_y;
        self.cal_points = 8 # number of points for the simple calibration


        # Order is: UPPERLEFT, UPPERRIGHT, LOWER RIGHT, (repeat for depth
        self.real_coord = np.float32([[self.min_x, self.max_y], [self.max_x,self.max_y], [self.max_x,self.min_y], [self.min_x, self.min_y],
                                      [self.min_x, self.max_y], [self.max_x,self.max_y], [self.max_x,self.min_y], [self.min_x, self.min_y]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0], [0.0, 0.0],
                                       [0.0, 0.0],[0.0, 0.0],[0.0, 0.0], [0.0, 0.0]])
        self.mouse_click_id = 0
        self.cal_flag = 0 # 0 - not calibrated, 1 - in progress, 2 - done
        # This transforms the image to global coordinates, arm is (0,0)

        self.aff_matrix = np.empty((2,3)) # affine matrix for simple calibration
        self.mat_rgb_to_real = np.empty((2,3))  # affine matrix for color to real
        self.mat_rgb_to_dep = np.empty((2,3))   # affine matrix for rgb to dep camera
        self.mat_dep_to_rgb = np.empty((2,3))
        self.mat_dep_abc = np.empty((1,3))      # affine matrix for depth to real

    def captureVideoFrame(self):
        """
        Capture frame from Kinect, format is 24bit RGB
        """
        self.currentVideoFrame = freenect.sync_get_video()[0]

    def captureDepthFrame(self):
        """
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        self.currentDepthFrame = freenect.sync_get_depth()[0]


    def loadVideoFrame(self):
        # self.currentVideoFrame = cv2.cvtColor(
        #     cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
        #     cv2.COLOR_BGR2RGB
        #     )
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/saved_task1.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        # self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)
        self.currentDepthFrame = cv2.imread("data/saved_task1_depth.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            img=QtGui.QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for QtGui
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)

            img=QtGui.QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertColorMask(self):
        color_mask = np.zeros(self.currentVideoFrame.shape, dtype=np.uint8)
        for key in colors:
            self.colorDetector(key)
            color_mask[np.where((self.colorMaskFrame > 0))] = colors[key]

        try:
            img=QtGui.QImage(color_mask,
                         color_mask.shape[1],
                         color_mask.shape[0],
                         QtGui.QImage.Format_RGB888
                         )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertDepthMask(self):
        depth_mask = np.zeros(self.currentVideoFrame.shape, dtype=np.uint8)
        for key in colors:
            self.blockDetector(key)
            color = np.array(colors[key][0][0], int)
            cv2.drawContours(depth_mask, self.blockContours, -1, color, 5)
            # depth_mask[np.where((self.depthMaskFrame > 0))] = colors[key]

        try:
            img=QtGui.QImage(depth_mask,
                         depth_mask.shape[1],
                         depth_mask.shape[0],
                         QtGui.QImage.Format_RGB888
                         )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertTest(self):
        depth_mask = self.currentVideoFrame.copy()
        for stack in xrange(1,0,-1):
            block_dict = {}
            block_dict = self.depthBlockDetector(stack)
            print block_dict
            for c in self.blockCenters:
                cv2.circle(depth_mask, (c[0], c[1]), 10, (255,255,255), 2)

        try:
            img=QtGui.QImage(depth_mask,
                         depth_mask.shape[1],
                         depth_mask.shape[0],
                         QtGui.QImage.Format_RGB888
                         )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None


    def loadCalibration(self):
        """
        TODO (OPTIONAL):
        Load csmera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """
        #pass
        self.mat_rgb_to_real = np.load('cal_rgb_to_real.npy')
        self.mat_rgb_to_dep = np.load('cal_rgb_to_dep.npy')
        self.mat_dep_to_reg = np.load('cal_dep_to_rgb.npy')
        self.mat_dep_abc = np.load('cal_dep_abc.npy')
        self.mat_dep_to_rgb = np.load('cal_dep_to_rgb.npy')
        self.mouse_coord = np.load('mouse_coord.npy')

    def pixelToCamera(self, rgb_x, rgb_y):
        rgb_pix_v = np.float32([rgb_x,rgb_y,1])
        dep = np.float32(np.dot(self.mat_rgb_to_dep, rgb_pix_v)) # depth coord to look for in depth image
        dep_x = int(dep[0])
        dep_y = int(dep[1])

        dep_val = self.currentDepthFrame[dep_y][dep_x]

        # real world coordinates for color
        rw = np.dot(self.mat_rgb_to_real, rgb_pix_v)

        Z_scale = 1.097 * 0.1236 * np.tan(dep_val/2842.5 + 1.1863) #1.097 * 0.1236 * np.tan(dep_val/2842.5 + 1.1863)
        # 0.1236 0.1373
        rw = Z_scale * rw
        Z_scale = -1000.0 * Z_scale * 3.7/4

        return np.array([rw[0], rw[1], Z_scale])

    def cameraToArm(self, camera_coord):
        xy1 = np.float32([camera_coord[0],camera_coord[1],1])
        Z = camera_coord[2] - self.mat_dep_abc.dot(xy1)
        return np.array([camera_coord[0], camera_coord[1], Z])

    def pixelToArm(self, x, y):
        camera_coord = self.pixelToCamera(x, y)
        return self.cameraToArm(camera_coord)

    # def getDepthMask(self, color_str = "red"):
    #     mask = np.empty((480,640), dtype=np.uint8)
    #     hull = cv2.convexHull(self.mouse_coord[0:4])
    #     self.colorDetector(color_str)
    #     total_x = 0.0
    #     total_y = 0.0
    #     total_num = 0.0
    #     for y in range(480):
    #         for x in range(640):
    #             # print cv2.pointPolygonTest(hull, (x, y), False)
    #             # print self.colorMaskFrame[y,x]
    #             if cv2.pointPolygonTest(hull, (x, y), False) > 0 and self.colorMaskFrame[y,x] > 0:
    #                 try:
    #                     armcoord = self.pixelToArm(x, y)
    #                     if (armcoord[2] > 20) and \
    #                         (np.abs(armcoord[0]) < 300) and \
    #                         (np.abs(armcoord[1]) < 300) and \
    #                         ((np.abs(armcoord[0]) > 60) or (np.abs(armcoord[1]) > 60)):
    #                         mask[y, x] = 255
    #                         total_x += x
    #                         total_y += y
    #                         total_num += 1
    #                     else:
    #                         mask[y, x] = 0
    #                 except:
    #                     mask[y, x] = 0
    #             else:
    #                 mask[y, x] = 0
    #     # print mask
    #     self.depthMaskFrame = mask

    #     # print "x: ", total_x/total_num, "y: ", total_y/total_num
    #     # print "arm coord: ", self.pixelToArm(int(total_x/total_num), int(total_y/total_num))

    #     # img = [m.tolist() for m in mask]
    #     # print img
    #     _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    #     # print len(contours)
    #     centres = []
    #     for i in range(len(contours)):
    #         moments = cv2.moments(contours[i])
    #         if(moments['m00'] == 0):
    #             continue
    #         centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
    #     # print centres

    #     # cv2.imshow('depth map', mask)
    #     # return self.pixelToArm(int(total_x/total_num), int(total_y/total_num))
    #     print "Center: {}".format(color_str)
    #     print [self.pixelToArm(c[0], c[1]) for c in centres]
    #     return [self.pixelToArm(c[0], c[1]) for c in centres]

    def filter_mask(self, img, hsv_colour, threshold):
        h = hsv_colour[0][0][0]
        s = hsv_colour[0][0][1]
        v = hsv_colour[0][0][2]

        gkernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(img, -1, gkernel)

        lower_bound = np.array([h - threshold,50,50])
        upper_bound = np.array([h + threshold,255,255])

        if np.all(hsv_colour == hsv_black):
            lower_bound = np.array([20,20,20])
            upper_bound = np.array([90,90,90])
            img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)

        mask = cv2.inRange(img, lower_bound, upper_bound)
        kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

        return mask

    def colorDetector(self, color_str="blue"):
        """
        TODO:
        Implement your block detector here.
        You will need to locate
        blocks in 3D space
        """

        rgb_color = colors[color_str]
        hsv_color = cv2.cvtColor(rgb_color, cv2.COLOR_RGB2HSV)
        hsv_img = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2HSV)
        mask = self.filter_mask(hsv_img, hsv_color, threshold)
        self.colorMaskFrame = mask;

    def nearestNeighborColor(self, img, width, length):
        min_distance = 99999999999
        color_fit = ""
        for color_str in colors:
            # print color_str
            # color_hsv = cv2.cvtColor(colors[color_str], cv2.COLOR_RGB2HSV)
            color_rgb = colors[color_str]
            current_distance = 0.0
            for x in xrange(width):
                for y in xrange(length):
                    # print img[y][x]
                    # pixel_hsv = cv2.cvtColor(np.array([[img[y][x]]]), cv2.COLOR_RGB2HSV)
                    pixel_rgb = np.array(img[y][x])
                    # print pixel_hsv
                    current_distance = current_distance + np.linalg.norm(pixel_rgb-color_rgb)
            print current_distance
            if current_distance < min_distance:
                min_distance = current_distance
                color_fit = color_str

        return color_fit

    def depthBlockDetector(self, stack_size = 1):
        # print stack_size
        raw_board = 715
        raw_stack_block = 15
        raw_depth_max = raw_board - raw_stack_block * stack_size

        depth_mask = cv2.inRange(self.currentDepthFrame, 500, raw_depth_max)

        kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_CLOSE, kernal)
        depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_OPEN, kernal)
        depth_mask = cv2.warpAffine(depth_mask, self.mat_dep_to_rgb, (640, 480))

        self.depthMaskFrame = depth_mask

        centers = self.contourDetector(depth_mask)

        # cv2.imshow("depth", depth_mask)
        # cv2.waitKey(0)

        block_dict = {"red": [], "blue": [], "green": [], "yellow": [], "black": []} #colors to array of coordiantes
        color_radius = 5
        self.blockCenters = []
        for c in centers:
            coord = self.pixelToArm(c[0], c[1])
            if (abs(coord[0]) < 300 and abs(coord[1]) < 300) and (abs(coord[0]) > 120 or abs(coord[1]) > 120):
                img = self.currentVideoFrame[(c[1]-color_radius):(c[1]+color_radius), (c[0]-color_radius):(c[0]+color_radius)]
                color_str = self.nearestNeighborColor(img, color_radius, color_radius)
                block_dict[color_str].append(coord)
                self.blockCenters.append(c)

        # print block_dict
        return block_dict


    def contourDetector(self, mask, saveContour = False):
        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        hull = cv2.convexHull(self.mouse_coord[0:4])

        centers = []
        if saveContour:
            self.blockContours = []

        for i in range(len(contours)):

            moments = cv2.moments(contours[i])

            # filter by area
            if(moments['m00'] == 0 or cv2.contourArea(contours[i]) < 110):
                continue

            x, y = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])

            # Filter out the blocks out side of the board
            if cv2.pointPolygonTest(hull, (x, y), False) < 0:
                continue
            centers.append((x, y))

            if saveContour:
                self.blockContours.append(contours[i])

        return centers

    # returns center of blocks in arm coordinates mm (x, y, z)
    def blockDetector(self, color_str = "green"):
        rgb_color = colors[color_str]
        hsv_color = cv2.cvtColor(rgb_color, cv2.COLOR_RGB2HSV)
        hsv_img = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2HSV)
        mask = self.filter_mask(hsv_img, hsv_color, threshold)

        self.colorMaskFrame = mask;

        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)

        hull = cv2.convexHull(self.mouse_coord[0:4])

        centers = self.contourDetector(mask, saveContour=True)

        # print color_str
        block_centers = []
        for c in centers:
            coord = self.pixelToArm(c[0], c[1])

            # filter by valid coordinates # Second line is filtering out the arm
            if (coord[0] > -300 and coord[0] < 0 and abs(coord[1]) < 300) \
            and (abs(coord[0]) > 120 or abs(coord[1]) > 120) \
            and coord[2] > 20 and coord[2] < 500:
                block_centers.append(coord)

        return block_centers

    # returns dictionary of color : array of coordinates of block center
    def getAllBlocks(self):
        block_dict = {} #colors to array of coordiantes
        for key in colors:
            print key
            coords = self.blockDetector(key)
            for coord in coords:
                print "x: {} y: {} z: {} (mm)".format(coord[0], coord[1], coord[2])
            color_dict = {key: coords}
            block_dict.update(color_dict)

        print "block_dict: ", block_dict
        return block_dict

    def getAffineTransform(self, MA, MB):
        """
        Least Square
        """
        n = MA.shape[0]
        A = np.zeros((n*2, 6))
        B = MB.reshape(2*n, 1)
        for i in range(n):
            A[2*i, 0:3] = np.append(MA[i], 1)
            A[2*i+1, 3:6] = np.append(MA[i], 1)

        return np.linalg.pinv(A).dot(B).reshape(2,3)
