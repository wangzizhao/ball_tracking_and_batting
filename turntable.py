import lcm
import time
import numpy as np
import sys
import math
sys.path.append("lcmtypes")
from math import *

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t
from lcmtypes import dynamixel_config_t
from lcmtypes import dynamixel_config_list_t
from lcmtypes.camera_coord_t import camera_coord_t

"""Useful Conversions"""
PI = np.pi
D2R = PI/180.0
R2D = 180.0/PI
ANGLE_TOL = 2*PI/180.0

# =========================================== Project begins =========================================== #
from scipy.signal import tf2ss
from scipy import signal
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# ============================================ Project ends ============================================ #

def clamp_radians(radians):
    '''
    clamp angle to the range (-PI, PI]
    '''
    while radians > PI:
        radians -= 2*PI

    while radians <= -PI:
        radians += 2*PI

    return radians

""" Turntable Class """
class Turntable():
    def __init__(self, id, dT=1/120.0):
        """
        id: turntable ID, 1 or 2
        dT: control period, unit: second
        """


        """ Commanded Values """
        self.num_joints = 2                         # number of motors, increase when adding gripper
        self.joint_angles = [0.0] * self.num_joints # radians

        # you must change this to an array to control each joint speed separately
        self.speed = [1.0, 1.0]                         # 0 to 1
        self.max_torque = [0.0, 0.0]                    # 0 to 1
        self.max_torque = [1.0, 1.0]                    # 0 to 1
        self.speed_multiplier = 1
        self.torque_multiplier = 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1
        self.load_fb = [0.0] * self.num_joints         # -1 to 1
        self.temp_fb = [0.0] * self.num_joints         # Celsius

        """ Waypoint Plan - TO BE USED LATER """
        self.plan = []
        self.plan_status = 0
        self.wpt_number = 0
        self.wpt_total = 0

        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()

        # ===================================== Project begins ===================================== #
        # For turn table control
        self.lost_sec = 0.5 # if lose tracking longer than self.lost_sec time, go to some preset pose
        self.lost_cnt = 0
        self.dT = dT

        self.error_x = 0
        self.error_y = 0
        self.prev_error_x = 0
        self.prev_error_y = 0

        self.int_x = 0
        self.int_y = 0

        ## For the camera coord calc
        ## the trans matrix from base to cam
        ## added by zhangdl
        self.M = []

        # For LCM comm with camera and turntable driver
        # print "Initialized: ", id
        self.id = id
        if self.id == 1:
            self.init_angles = [45*D2R, 90*D2R]
            lcmMotorSub = self.lc.subscribe("DXL_STATUS_REX1",
                                        self.feedback_handler)
            lcmCameraSub = self.lc.subscribe("CAMERA_COORD1",
                                        self.camera_feedback_handler)
            self.DXL_COMMAND = "DXL_COMMAND_REX1"
            self.DXL_CONFIG  = "DXL_CONFIG_REX1"
        elif self.id == 2:
            self.init_angles = [-45*D2R, 90*D2R]
            lcmMotorSub = self.lc.subscribe("DXL_STATUS_REX2",
                                        self.feedback_handler)
            lcmCameraSub = self.lc.subscribe("CAMERA_COORD2",
                                        self.camera_feedback_handler)
            
            self.DXL_COMMAND = "DXL_COMMAND_REX2"
            self.DXL_CONFIG  = "DXL_CONFIG_REX2"
        else:
            print "Wrong rex id!!!! Must be 1 or 2!!!!"
            assert(0)

        self.joint_angles = np.copy(self.init_angles)
        lcmMotorSub.set_queue_capacity(1)
        lcmCameraSub.set_queue_capacity(1)

        self.cfg_publish_default()
        self.cmd_publish()
        """ Arm Lengths """
        self.base_len     = 116.2
        self.shoulder_len = 78#90 #78# important! 93.2

        """ DH Table """
        # radians
        self.dh_table = [{"d" : self.base_len, "a" : 0,                 "alpha": PI/2}, \
                         {"d" : 0,             "a" : self.shoulder_len, "alpha": 0   }]
        self.last_time = 0

        # lowpass filter
        cutoff = 3
        order = 2
        b, a = signal.butter(order, 2*cutoff*self.dT, 'low', analog=False)
        self.A, self.B, self.C, self.D = tf2ss(b, a)
        self.A, self.B, self.C, self.D = np.array(self.A), np.array(self.B.T), np.array(self.C), np.array(self.D)
        self.B = self.B[0]
        self.C = self.C[0]
        # print self.A
        # print self.B
        # print self.C
        # print self.D
        self.x_vec = np.zeros(order)
        self.y_vec = np.zeros(order)

        self.calm_down_count = 0
        # ====================================== Project ends ====================================== #
        

    def cmd_publish(self):
        """
        Publish the commands to the arm using LCM.
        This publishes the joint angles currently set in the Rexarm object
        You need to call this function to command the arm.
        """
        msg = dynamixel_command_list_t()
        msg.len = self.num_joints
        # self.clamp()

        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            cmd.speed = self.speed_multiplier * self.speed[i]
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
            # print "self.joint_angles: ", self.joint_angles[i], " self.speed: ", self.speed[i]
        self.lc.publish(self.DXL_COMMAND,msg.encode())

    def cfg_publish_default(self):
        """
        Publish default configuration to arm using LCM.
        """
        msg = dynamixel_config_list_t()
        msg.len = self.num_joints
        for i in range(msg.len):
            cfg = dynamixel_config_t()
            cfg.utime = int(time.time() * 1e6)
            cfg.kp = 250
            cfg.ki = 0
            cfg.kd = 0
            cfg.compl_margin = 0
            cfg.compl_slope = 16
            cfg.LED = 1
            msg.configs.append(cfg)
        self.lc.publish(self.DXL_CONFIG,msg.encode())

    def get_feedback(self):
        """
        LCM Handler function
        Called continuously with timer by control_station.py
        times out after 10ms
        """
        self.lc.handle_timeout(10)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = dynamixel_status_list_t.decode(data)

        for i in range(msg.len):
            
            self.joint_angles_fb[i] = msg.statuses[i].position_radians
            if i == 0:
                if self.id == 1:
                    self.joint_angles_fb[i] -= 0
                elif self.id == 2:
                    self.joint_angles_fb[i] += 0.02
                # print "id: ", self.id, 'angle: ', self.joint_angles_fb[i] * R2D
            self.speed_fb[i] = msg.statuses[i].speed
            self.load_fb[i] = msg.statuses[i].load
            self.temp_fb[i] = msg.statuses[i].temperature

        # print "time interval Hz: ", 1000000.0/(msg.statuses[-1].utime - self.last_time)
        self.last_time = msg.statuses[-1].utime

    def camera_feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """

        msg = camera_coord_t.decode(data)
        if msg.x == -1:
            self.lost_cnt += 1
        else:
            self.error_x = msg.x - FRAME_WIDTH/2
            self.error_y = -(msg.y - FRAME_HEIGHT/2)
            self.lost_cnt = 0
            # print "error_x:", self.error_x, ", error_y:", self.error_y

    def calc_A_FK(self, theta, link):
        """
        theta is radians of the link number
        link is 0 indexed (0 is base, 1 is shoulder ...)
        returns a matrix A(2D array)
        """
        i = link

        d = self.dh_table[i]["d"]
        a = self.dh_table[i]["a"]
        alpha = self.dh_table[i]["alpha"]

        A = [[cos(theta), -sin(theta)*cos(alpha), sin(theta)*cos(alpha),  a*cos(theta)], \
            [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)], \
            [0,          sin(alpha),             cos(alpha),             d           ], \
            [0,          0,                      0,                      1           ]]

        # A_inv = [[              cos(theta),             sin(theta),           0,            -a], \
        #       [ -cos(alpha)*sin(theta),  cos(alpha)*cos(theta),  sin(alpha), -d*sin(alpha)], \
        #       [  sin(alpha)*sin(theta), -sin(alpha)*cos(theta),  cos(alpha), -d*cos(alpha)], \
        #       [                      0,                      0,           0,             1]]

        # assert(np.abs(np.linalg.det(np.dot(A, A_inv)) - 1) < 1e-5)

        return A

    def rexarm_FK(self, link=1, cfg=1):
        """
        TODO: implement this function

        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the
        desired link
        """
        # point of endpoint

        # iterate in reverse and accumulate multiply with point
        self.get_feedback()

        Trans = np.identity(4)
        for i in xrange(link, -1, -1):
            theta = self.joint_angles_fb[i]
            if i == 0:
                theta = theta + PI/2
            if i == 1:
                theta = theta + PI/2

            A = self.calc_A_FK(theta, i);
            Trans = np.matmul(A, Trans)

        phi = self.joint_angles_fb[0]
        theta = self.joint_angles_fb[1]
        factor1 = 0.95
        factor2 = 2.5
        factor1 = 1
        factor2 = 1
        if self.id == 1:
            h_dev = 0.05991 * factor1
        else:
            h_dev = 0.05991 * factor2
        B = np.array([[ np.cos(phi),-np.sin(phi), 0],
                      [ np.sin(phi), np.cos(phi), 0],
                      [           0,           0, 1]])
        C = np.array([[1,             0,              0],
                      [0, np.cos(theta),-np.sin(theta)],
                      [0, np.sin(theta), np.cos(theta)]])
        D = np.array([[-1,  0, 0],
                      [ 0, -1, 0],
                      [ 0,  0, 1]])
        E = np.array([[ np.cos(h_dev), 0, np.sin(h_dev)],
                      [             0, 1,             0],
                      [-np.sin(h_dev), 0, np.cos(h_dev)]])
        # E = np.eye(3)
        Trans[0:3, 0:3] = np.dot(B, np.dot(C, np.dot(D, E)))
        # print 
        # print '--------------------', Trans[0:3, 3]
        self.M = Trans

        Trans_inv = np.identity(4)
        Trans_inv[0:3, 3] = -Trans[0:3, 3]

        B = np.array([[ np.cos(phi),np.sin(phi), 0],
                      [ -np.sin(phi), np.cos(phi), 0],
                      [           0,           0, 1]])
        C = np.array([[1,             0,              0],
                      [0, np.cos(theta),np.sin(theta)],
                      [0, -np.sin(theta), np.cos(theta)]])
        D = np.array([[-1,  0, 0],
                      [ 0, -1, 0],
                      [ 0,  0, 1]])
        E = np.array([[ np.cos(h_dev), 0, -np.sin(h_dev)],
                      [             0, 1,              0],
                      [ np.sin(h_dev), 0,  np.cos(h_dev)]])
        # E = np.eye(3)
        BCD_inv = np.dot(E, np.dot(D, np.dot(C, B)))
        Trans_inv[0:3, 3] = BCD_inv.dot(Trans_inv[0:3, 3])
        Trans_inv[0:3, 0:3] = BCD_inv

        # print Trans.dot(Trans_inv)
        return Trans, Trans_inv

    def clamp(self):
        """
        TODO: implement this function

        Clamp Function
        Limit the commanded joint angles to ones physically possible
        so the arm is not damaged.
        """
        self.joint_angles = np.clip(self.joint_angles, [-179*D2R, 0], [179*D2R, 100 * D2R])
        self.speed = np.clip(np.abs(self.speed), [0.0, 0.0], [1.0, 1.0])         # 0 to 1

    def update(self):
        """
        Control servo based on ball's pos in the image
        """
        self.get_feedback()
        # self.calm_down_count += 1
        # if self.calm_down_count < 0.25/self.dT:
        #   self.speed = [0.5, 0.5]
        #   self.joint_angles = np.copy(self.init_angles)
        #   self.clamp()
        #   self.cmd_publish()
        #   return
        if self.lost_cnt < self.lost_sec/self.dT:
            self.KP = 0.002
            self.KI = 0
            self.KD = 0.0000

            D_x = (self.error_x - self.prev_error_x)/self.dT
            self.x_vec = np.dot(self.A, self.x_vec) + D_x * self.B
            D_x = np.dot(self.C, self.x_vec) + D_x * self.D

            D_y = (self.error_y - self.prev_error_y)/self.dT
            self.y_vec = np.dot(self.A, self.y_vec) + D_y * self.B
            D_y = np.dot(self.C, self.y_vec) + D_y * self.D

            # print 'D_x: ', int(D_x[0]), ' D_y: ', int(D_y[0])

            v_x = -(self.KP*self.error_x + self.KI*self.int_x + self.KD*D_x[0])
            v_y = -(self.KP*self.error_y + self.KI*self.int_y + self.KD*D_y[0])

            self.int_x += self.error_x * self.dT
            self.int_y += self.error_y * self.dT
            self.prev_error_x = self.error_x
            self.prev_error_y = self.error_y

            if v_x > 0:
                target_x = PI
                if self.id == 2 and self.joint_angles_fb[0] > 0:
                    target_x = 0
                # if self.id == 1:
                #   target_x = PI
                # elif self.id == 2:
                #   target_x = 0
            else:
                target_x = -PI
                if self.id == 1 and self.joint_angles_fb[0] < 0:
                    target_x = 0
                # elif self.id == 2:
                #   target_x = -PI

            if v_y > 0:
                target_y = 120 * D2R
            else:
                target_y = 0

            self.speed = [v_x, v_y]
            self.joint_angles = [target_x, target_y]
        else:
            self.speed = [0.2, 0.2]
            self.joint_angles = np.copy(self.init_angles)
            # self.calm_down_count = 0
            self.clamp()
            self.cmd_publish()
            while np.linalg.norm(np.subtract(self.joint_angles_fb, self.joint_angles)) *R2D > 2:
                self.get_feedback()

        self.clamp()
        self.cmd_publish()

    ## For the camera coord calc
    ## the trans matrix from base to cam
    ## added by zhangdl 

    def rexarm_FK_loop(self):
        while True:
            self.rexarm_FK()
            
    def get_M(self):
        return self.M






