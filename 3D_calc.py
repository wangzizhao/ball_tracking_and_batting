import cv2
import time
import numpy as np
# for the lcm communication for camera
import sys
import lcm
sys.path.append("lcmtypes")
from lcmtypes.camera_coord_t import camera_coord_t

# the following is from the turnable to get he xyz
from turntable import Turntable

# =============================================
#               Constants                     #
# =============================================
csv_filename = "kalman6.csv"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_CHANNEL_1 = "CAMERA_COORD1"
CAMERA_CHANNEL_2 = "CAMERA_COORD2"


# trans matrix from base of 
# cam1 to cam2
# [[   1.    0.    0.  -1800.]
#  [   0.    1.    0.    0.]
#  [   0.    0.    1.    0.]
#  [   0.    0.    0.    1.]]
MAT_CAM1toCAM2 = np.eye(4, dtype=float)
MAT_CAM1toCAM2[0][3] = -1840.0
MAT_CAM1toCAM2[2][3] = 0.0

# the new link is 77mm
# 99.4 80 640  = 795.2 Mar 31, 04:22
FOCAL_LENGTH_1 =  526.235 #* 117/114 #732.4 #423.5
FOCAL_LENGTH_2 = FOCAL_LENGTH_1 * 103.5 / 102.8
X_PIXEL_OFFSET = 0 #-32.0

# constant for kalman filter
COORD_CHANNEL = "COORD_CHANNEL"
GRAVITY = 9.8 * np.array([0,0,1])
POS_COEF1 = 0.5
VEL_COEF1 = 0.5
POS_COEF2 = 0.5
VEL_COEF2 = 0.5
TIME_INTERVAL = 1.0/100.0
MODE_SWITCH_THRES = 30
Y_MIN = -900
Y_MAX = 1600

# mode 
INITIAL_MODE = 0
PREPARE_MODE = 1
DETECT_MODE = 2
FINISH_MODE = 3

X_OFFSET = -1138.754 -52
Y_OFFSET = -1460.1108 -46.5
Z_OFFSET = 37

# =============================================
#               Variables                     #
# =============================================

msg_1 = camera_coord_t()
msg_2 = camera_coord_t()

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

def calc_RT_matrix(M1, M1_inv, M2, M2_inv):
    """
    @param:
        M1 is the trans and rot matrix from base1 to cam1
        M2 is the trans and rot matrix from base2 to cam2
    @return:
        Rb Tb, rotation matrix and translation matrix from cam1 to cam2
    """
    MAT = np.matmul(M2_inv, np.matmul(MAT_CAM1toCAM2, M1))
    Rb = MAT[0:3, 0:3]
    Tb = MAT[0:3, 3]

    return Rb, Tb

def trans_camera_to_base(T, M):
    """
    @param:
        T is the xyz pos in cam coordinat
        M is the trans and rot matrix from base to cam2
    """
    cam_vec = np.array([[T[0]],[T[1]],[T[2]],[1]], dtype="float64")
    base_pos = np.matmul(M, cam_vec)
    return np.transpose(base_pos[0:3, :])


def kalman_filter1(old_state, new_state, input_valid = True):
    """
        Fitting with the model for the target flying in the air
    """
    drag_coff = (0.5*0.445*1.227/1e6*np.pi*(40/2)**2/2.7)
    state = np.zeros(6)

    if input_valid:
        state[0:3] = POS_COEF1 * new_state \
            + (1 - POS_COEF1) * (old_state[0:3] + old_state[3:6] * TIME_INTERVAL)
        friction = drag_coff * np.linalg.norm(old_state[3:6]) * old_state[3:6]
        state[3:6] = VEL_COEF1 * (new_state - old_state[0:3]) / TIME_INTERVAL \
            + (1 - VEL_COEF1) * (old_state[3:6] - (GRAVITY + friction) * TIME_INTERVAL)
    else:
        state[0:3] = old_state[0:3] + old_state[3:6] * TIME_INTERVAL
        friction = drag_coff * np.linalg.norm(old_state[3:6]) * old_state[3:6]
        state[3:6] = old_state[3:6] - (GRAVITY + friction) * TIME_INTERVAL

    return state

# def kalman_filter2(old_state, new_state, input_valid = True):
#     """
#         Fitting with the model for the target with no accerlation
#     """
#     state = np.zeros(6)
#     if input_valid:
#         state[0:3] = POS_COEF2 * new_state \
#             + (1 - POS_COEF2) * (old_state[0:3] + old_state[3:6] * TIME_INTERVAL)
#         state[3:6] = VEL_COEF2 * (new_state - old_state[0:3]) / TIME_INTERVAL \
#             + (1 - VEL_COEF2) * old_state[3:6]
#     else:
#         state[0:3] = old_state[0:3] + old_state[3:6] * TIME_INTERVAL
#         state[3:6] = old_state[3:6]
#     return state

def calc_pos_in_camera_coord(Rb, Tb, ue1, ve1, ue2, ve2, f1=FOCAL_LENGTH_1, f2=FOCAL_LENGTH_2):
    """
    @param:
        Rb and Tb is the rotation matrix and translation matrix from cam1 to cam2
        ue1, ve1 is the x and y in the cam1 coordinates
        ue2, ve2 is the x and y in the cam2 coodiantes
        f is the focal length and scalar factor of the camera. 
    @return:
        the [x,y,z] pos in the cam1 coordinate and cam2 coodinates
    """
    A1 = np.array([[ (ue1-CAMERA_WIDTH/2) /f1], [ (ve1-CAMERA_HEIGHT/2) /f1], [1]])
    A2 = -np.matmul(Rb, A1)
    A3 = np.array([[ (ue2-CAMERA_WIDTH/2) /f2], [ (ve2-CAMERA_HEIGHT/2) /f2], [1]])
    A = np.concatenate((A2, A3), axis=1)
    # T = [Tz1, Tz2] z-vector with first and second cameras
    T = np.matmul(np.linalg.pinv(A), Tb)
    # the translation vector from the ball to the first camera
    T1 = np.array([ (ue1-CAMERA_WIDTH/2) *T[0]/f1, (ve1-CAMERA_HEIGHT/2) *T[0]/f1, T[0]])
    # the translation vector from the ball to the second camera
    T2 = np.array([ (ue2-CAMERA_WIDTH/2) *T[1]/f2, (ve2-CAMERA_HEIGHT/2) *T[1]/f2, T[1]])

    return T1, T2

# Correct
def camera_feedback_handler(channel, data):
    """
    Feedback Handler for LCM from CAMERA_COORD1
    this is run when a feedback message is recieved
    """
    msg = camera_coord_t.decode(data)
    if channel == CAMERA_CHANNEL_1:
        # msgs_1.append(msg)
        msg_1.x = msg.x
        msg_1.y = msg.y
        msg_1.z = msg.z
        msg_1.v_x = msg.v_x
        msg_1.v_y = msg.v_y
        msg_1.v_z = msg.v_z
        msg_1.mode = msg.mode
        msg_1.utime = msg.utime
        
        # test lcm
        # while True:
        #     img = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), np.uint8)
        #     cv2.circle(img,(int(msg_1.x),int(msg_1.y)), 10, (0,0,255), -1)
        #     cv2.line(img, (int(CAMERA_WIDTH / 2), 0), (int(CAMERA_WIDTH / 2), CAMERA_HEIGHT), (0,255,0), 1)
        #     cv2.line(img, (0, int(CAMERA_HEIGHT / 2)), (int(CAMERA_WIDTH), int(CAMERA_HEIGHT / 2)), (0,255,0), 1)
        #     cv2.imshow("img1", img)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break

        # print("msg: ({}, {}, {})".format(1, round(msg.x), round(msg.y)))
    elif channel == CAMERA_CHANNEL_2:
        # msgs_2.append(msg)
        msg_2.x = msg.x
        msg_2.y = msg.y
        msg_2.z = msg.z
        msg_2.v_x = msg.v_x
        msg_2.v_y = msg.v_y
        msg_2.v_z = msg.v_z
        msg_2.mode = msg.mode
        msg_2.utime = msg.utime
        # msg_2.z = msg.z
        # print("msg: ({}, {}, {})".format(2, round(msg.x), round(msg.y)))

        # test lcm
        # while True:
        #     img = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), np.uint8)
        #     cv2.circle(img,(int(msg_2.x),int(msg_2.y)), 10, (0,0,255), -1)
        #     cv2.line(img, (int(CAMERA_WIDTH / 2), 0), (int(CAMERA_WIDTH / 2), CAMERA_HEIGHT), (0,255,0), 1)
        #     cv2.line(img, (0, int(CAMERA_HEIGHT / 2)), (int(CAMERA_WIDTH), int(CAMERA_HEIGHT / 2)), (0,255,0), 1)
        #     cv2.imshow("img2", img)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break

if __name__ == '__main__':
    turnable1 = Turntable(1)
    turnable2 = Turntable(2)

    lc = lcm.LCM()
    lcmCameraSub1 = lc.subscribe(CAMERA_CHANNEL_1, camera_feedback_handler)
    lcmCameraSub2 = lc.subscribe(CAMERA_CHANNEL_2, camera_feedback_handler)
    lcmCameraSub1.set_queue_capacity(1)
    lcmCameraSub2.set_queue_capacity(1)
    lc.handle_timeout(5)


    # at first mode = -1, no camera see the ball
    # meaning of mode:
    #   -1: Unintialized
    #    0: Y < Y_MIN
    #    1: Y_MIN < Y < Y_MAX
    #    2: Y > Y_MAX
    coord_res = camera_coord_t()
    mode = INITIAL_MODE 

    target_state = np.zeros(6)
    prev_target_state = np.zeros(3)
    save_data = []
    is_saved = False
    in_sight_count = 0
    detect_begin = time.time()

    while True:
        iteration_start = time.time()
        
        # send msg 
        coord_res.x = target_state[0] + X_OFFSET
        coord_res.y = target_state[1] + Y_OFFSET
        coord_res.z = target_state[2] + Z_OFFSET
        coord_res.v_x = target_state[3]
        coord_res.v_y = target_state[4]
        coord_res.v_z = target_state[5]
        coord_res.mode = mode
        coord_res.utime = int((time.time() - detect_begin) * 1e6)
        print 'xyz,', target_state[:3], ', mode', coord_res.mode
        lc.publish(COORD_CHANNEL, coord_res.encode())

        # start working 
        lc.handle_timeout(5)

        if mode == FINISH_MODE:
            if not is_saved:
                np.savetxt(csv_filename, np.array(save_data), delimiter=",")
                is_saved = True
            else:
                mode = INITIAL_MODE
                
        if mode == INITIAL_MODE:
            target_state = np.zeros(6)
            prev_target_state = np.zeros(3)
            save_data = []
            is_saved = False
            in_sight_count = 0
            detect_begin = time.time()

        M1, M1_inv = turnable1.rexarm_FK()
        M2, M2_inv = turnable2.rexarm_FK()
        
        Rb, Tb = calc_RT_matrix(M1, M1_inv, M2, M2_inv)

        if not (msg_1.x != -1 and msg_2.x != -1):   # at least one camera missed
            if mode == DETECT_MODE:     # if detect mode, use the kalman filter
                target_state = kalman_filter1(target_state, target_state[0:3], False)
                if target_state[1] > Y_MAX: # predict y is larger than Y_MAX
                    mode = FINISH_MODE
                if target_state[1] < Y_MIN - 300:
                    mode = INITIAL_MODE
            continue

        # two cameras both detect                 
        T1, T2 = calc_pos_in_camera_coord(Rb, Tb, msg_1.x, msg_1.y, msg_2.x, msg_2.y)
        base1 = trans_camera_to_base(T1, M1)
        X = base1[0][0]
        if X < 500 or X > 1500:
            if mode == DETECT_MODE:     # if detect mode, use the kalman filter
                target_state = kalman_filter1(target_state, target_state[0:3], False)
                if target_state[1] > Y_MAX: # predict y is larger than Y_MAX
                    mode = FINISH_MODE
                if target_state[1] < Y_MIN - 300:
                    mode = INITIAL_MODE
            continue

        if mode == INITIAL_MODE:
            if base1[0][1] > Y_MIN: # haven't gone back to the iteration_starting line
                continue
            else:
                mode = PREPARE_MODE
                continue

        elif mode == PREPARE_MODE:
            if base1[0][1] > Y_MIN:
                in_sight_count += 1
            else:
                in_sight_count = 0

            if in_sight_count == 5:
                prev_target_state = list(base1)

            if in_sight_count > 5:  # 3 counts for base[0][1] > Y_MIN
                mode = DETECT_MODE
                detect_begin = time.time()
                target_state[0:3] = base1  # first one, dont consider the speed
                target_state[3:6] = (base1 - prev_target_state) / TIME_INTERVAL

        elif mode == DETECT_MODE:   ## first target_state is recorded, calculate the speed in detect
            target_state = kalman_filter1(target_state, base1)
            X = base1[0][0]

            if target_state[4] < 0:
                print " = = = = = = = = = OH MY YYYY +++++++++"

            if X < 500 or X > 1500:
                print "NO!!! Bad X, base1", base1
            save_data.append(list(target_state) + list(base1[0]) + [int(time.time() * 1e6)])

            if base1[0][1] > Y_MAX:
                mode = FINISH_MODE
            if target_state[1] < Y_MIN - 300:
                mode = INITIAL_MODE

        elif mode == FINISH_MODE:
            # already handle in the beginning
            pass


        elapse = time.time() - iteration_start
        if elapse < TIME_INTERVAL:
            time.sleep(TIME_INTERVAL - elapse)
        else:
            print "NO!!! To much computation time each loop!!!"