import cv2
import time
import numpy as np
# for the lcm communication for camera
import sys
import lcm
sys.path.append("../lcmtypes")
from lcmtypes.camera_coord_t import camera_coord_t

# the following is from the turnable to get he xyz
sys.path.append("../")
import turnable

msgs_1 = []
msgs_2 = []

CMAERA_CHANNEL_1 = "CAMERA_COORD1"
CMAERA_CHANNEL_2 = "CAMERA_COORD2"

# trans matrix from base of cam1 to cam2
MAT_CAM1toCAM2 = np.eye(4, dtype=float)
MAT_CAM1toCAM2[0][3] = 500 

def calc_RT_matrix(M1, M2):
    """
    @parrm:
        M1 is the trans and rot matrix from base1 to cam1
        M2 is the trans and rot matrix from base2 to cam2
    @return:
        Rb Tb, rotation matrix and translation matrix fromc cam1 to cam2
    """
    # MAT = M2 * MAT_CAM1toCAM2 * M1.inv
    MAT = np.matmul(M2, np.matmul(MAT_CAM1toCAM2, np.linalg.inv(M1)))

    Rb = MAT[0:3, 0:3]
    Tb = MAT[0:3, 3]

    return Rb, Tb



def calc_pos_in_camera_coord(Rb, Tb, ue1, ve1, ue2, ve2, f):
    """
    @param:
        Rb and Tb is the rotation matrix and translation matrix from cam1 to cam2
        ue1, ve1 is the x and y in the cam1 coordinates
        ue2, ve2 is the x and y in the cam2 coodiantes
        f is the focal length and scalar factor of the camera. 
    @return:
        the [x,y,z] pos in the cam1 coodinate and cam2 coodinates
    """
    A1 = np.array([[ue1/f], [ve1/f], [1]])
    A2 = -np.matmul(Rb, A1)
    A3 = np.array([[ue2/f], [ve2/f], [1]])
    A = np.concatenate((A2, A3), axis=1)
    # T = [Tz1, Tz2] z-vector with first and second cameras
    T = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), Tb)
    # the translation vector from the ball to the first camera
    T1 = np.array([ue1*T[0]/f,ve1*T[0],T[0]])
    # the translation vector from the ball to the second camera
    T2 = np.array([ue2*T[1]/f,ve2*T[1],T[1]])

    return T1, T2


def camera_feedback_handler(channel, data):
    """
    Feedback Handler for LCM from CAMERA_COORD1
    this is run when a feedback message is recieved
    """
    msg = camera_coord_t.decode(data)
    if channel == CMAERA_CHANNEL_1:
        msgs_1.append(msg)
        print("msg: ({}, {}, {}, {})".format(1, msg.x, msg.y, msg.radius) )
    elif channel == CMAERA_CHANNEL_2:
        msgs_2.append(msg)
        print("msg: ({}, {}, {})".format(2, msg.x, msg.y, msg.radius) )

if __name__ == '__main__':
    turnable1 = Turntable(1)
    turnable2 = Turntable(2)
    lc = lcm.LCM()
    lcmMotorSub1 = lc.subscribe(CMAERA_CHANNEL_1, camera_feedback_handler)
    lcmMotorSub2 = lc.subscribe(CMAERA_CHANNEL_2, camera_feedback_handler)
    while True:
        M1 = turnable1.rexarm_FK()
        M2 = turnable2.rexarm_FK()
        Rb, Tb = calc_RT_matrix(M1, M2)
        print "Tb: ", Tb
        print "Rb: ", Rb
        lc.handle()
        # # Create a black image
        # img = np.zeros((512,512,3), np.uint8)
        # cv2.circle(img,(int(msgs_1[-1].x),int(msgs_1[-1].y)), int(msgs_1[-1].radius), (0,0,255), -1)
        # cv2.imshow("img", img)
        # # test code 
        # if len(msgs_1) != 0 and len(msgs_1) == len(msgs_2):
        #   pos1, pos2 = calc_pos_in_camera_coord(Rb, Tb, msgs_1[-1].x, msgs_2[-1].y, 
        #       msgs_2[-1].x, msgs_2[-1].y, f)


