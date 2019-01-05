import cv2
import time
import numpy as np
# for the lcm communication for camera
import sys

CAMERA_LENGTH = 640
CAMERA_WIDTH = 480
CMAERA_CHANNEL_1 = "CAMERA_COORD1"
CMAERA_CHANNEL_2 = "CAMERA_COORD2"

# trans matrix from base of cam1 to cam2
# [[   1.    0.    0.  500.]
#  [   0.    1.    0.    0.]
#  [   0.    0.    1.    0.]
#  [   0.    0.    0.    1.]]
MAT_CAM1toCAM2 = np.eye(4, dtype=float)
MAT_CAM1toCAM2[0][3] = -500.0

FOCAL_LENGTH = 423.5

class Coord_calculator(object):
	"""docstring for Coord_calculator"""
	# def __init__(self, arg):
	# 	self.arg = arg

	def calc_RT_matrix(self, M1, M2):
	    """
	    @param:
	        M1 is the trans and rot matrix from base1 to cam1
	        M2 is the trans and rot matrix from base2 to cam2
	    @return:
	        Rb Tb, rotation matrix and translation matrix from cam1 to cam2
	    """
	            # test code for M1 and M2
	        # M1 = [[   0,    -1,    0,  1],
	        #      [   1,    0,    0,  10],
	        #      [   0,    0,    1,  100],
	        #      [   0,    0,    0,  1]]

	        # M2 = [[   1,    0,    0,  0],
	        #      [   0,    1,    0,  0],
	        #      [   0,    0,    1,  0],
	        #      [   0,    0,    0,  1]]
	    # MAT = M1.inv * M2 * MAT_CAM1toCAM2
	    MAT = np.matmul(np.linalg.inv(M2),np.matmul(MAT_CAM1toCAM2, M1))
	    Rb = MAT[0:3, 0:3]
	    Tb = MAT[0:3, 3]

	    # print "np.linalg.inv(M1): ", np.linalg.inv(M1)
	    # print "np.matmul(MAT_CAM1toCAM2, np.linalg.inv(M1)): ", np.matmul(MAT_CAM1toCAM2, np.linalg.inv(M1))
	    # print "np.linalg.inv(MAT_CAM1toCAM2): ", np.linalg.inv(MAT_CAM1toCAM2)
	    # print "MAT: ", MAT
	    # print "MAT_CAM1toCAM2: ", MAT_CAM1toCAM2
	    # print "M1: ", M1
	    # print "M2: ", M2

	    return Rb, Tb

	def trans_camera_to_base(self, T, M):
	    """
	    @param:
	        T is the xyz pos in cam coordinat
	        M is the trans and rot matrix from base to cam2
	    """
	    cam_vec = np.array([[T[0]],[T[1]],[T[2]],[1]])
	    base_pos = np.matmul(M, cam_vec)
	    return np.transpose(base_pos[0:3, :])


	def calc_pos_in_camera_coord(self, Rb, Tb, ue1, ve1, ue2, ve2, f=FOCAL_LENGTH):
	    """
	    @param:
	        Rb and Tb is the rotation matrix and translation matrix from cam1 to cam2
	        ue1, ve1 is the x and y in the cam1 coordinates
	        ue2, ve2 is the x and y in the cam2 coodiantes
	        f is the focal length and scalar factor of the camera. 
	    @return:
	        the [x,y,z] pos in the cam1 coordinate and cam2 coodinates
	    """
	    # print ue1-CAMERA_LENGTH/2, ve1-CAMERA_WIDTH/2, ue2-CAMERA_LENGTH/2, ve2-CAMERA_WIDTH/2
	    A1 = np.array([[(ue1-CAMERA_LENGTH/2)/f], [(ve1-CAMERA_WIDTH/2)/f], [1]])
	    # print A1
	    A2 = -np.matmul(Rb, A1)
	    A3 = np.array([[(ue2-CAMERA_LENGTH/2)/f], [(ve2-CAMERA_WIDTH/2)/f], [1]])
	    A = np.concatenate((A2, A3), axis=1)
	    # T = [Tz1, Tz2] z-vector with first and second cameras
	    #print np.matmul(A.T, A)
	    T = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), Tb)
	    # print T
	    # the translation vector from the ball to the first camera
	    T1 = np.array([(ue1-CAMERA_LENGTH/2)*T[0]/f,(ve1-CAMERA_WIDTH/2)*T[0]/f,T[0]])
	    # the translation vector from the ball to the second camera
	    T2 = np.array([(ue2-CAMERA_LENGTH/2)*T[1]/f,(ve2-CAMERA_WIDTH/2)*T[1]/f,T[1]])

	    return T1, T2