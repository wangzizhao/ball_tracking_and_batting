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

from scipy.integrate import ode
from scipy.optimize import curve_fit

"""Useful Conversions"""
PI = np.pi
D2R = PI/180.0
R2D = 180.0/PI
ANGLE_TOL = 2*PI/180.0
V_FB2RAD_S = 1024 * 0.111 * 2*np.pi/60.0

FINAL_BASE_ANGLE = -45 * D2R
RESPOND_TIME = 0.4
SPEED_FACTOR = 1.5
SPEED_ADDER = 0.1
RISE_TIME = 0.2
Z_height = 0.3

def fun_vy(t, p):
    return 1/0.1269/(p + t)

def fun_vz(t, p):
    return 8.78784 * np.tan(p - 1.1158 * t)

def ode_f(t, y):
    g = 9.8
    c = 0.1269
    v_sqrt = np.linalg.norm(y[-3:])
    return [y[3], y[4], y[5], -c*y[3]*v_sqrt, -c*y[4]*v_sqrt, -c*y[5]*v_sqrt - g]

def clamp_radians(radians):
    '''
    clamp angle to the range (-PI, PI]
    '''
    while radians > PI:
        radians -= 2*PI

    while radians <= -PI:
        radians += 2*PI

    return radians

def IK_is_clamped(IK_joint_angle):
    JA = np.minimum(np.asarray(IK_joint_angle), [PI, 120 * D2R, 120 * D2R, 120 * D2R])
    JA = np.maximum(JA, [-1 * PI, -120 * D2R, -120 * D2R, -120 * D2R])
    if np.array_equal(np.asarray(IK_joint_angle), JA):
        return False
    else:
        return True



""" Rexarm Class """
class Batter():
    def __init__(self):

        """ TODO: modify this class to add functionality you need """

        """ Commanded Values """
        self.num_joints = 6                         # number of motors, increase when adding gripper
        self.joint_angles = [0.0] * self.num_joints # radians

        self.upper_angles = [179*D2R, 120 * D2R, 120 * D2R, 120 * D2R, 150*D2R, 90 * D2R]
        self.lower_angles = [-179*D2R, -120 * D2R, -120 * D2R, -120 * D2R, 0* D2R, -90 * D2R]

        self.ready_pose = [-135*D2R, 60*D2R, 0*D2R, 0*D2R, 90*D2R, 0*D2R]
        # self.ready_pose = [-180*D2R, 60*D2R, 0*D2R, 0*D2R, 90*D2R, -45*D2R] #-135*D2R
        self.hit_pose = [-90*D2R, 60*D2R, 60*D2R, -60*D2R, 90*D2R, 0*D2R]
        # self.hit_pose = [-90*D2R, 60*D2R, 60*D2R, -60*D2R, 90*D2R, 0*D2R]

        # you must change this to an array to control each joint speed separately
        self.speed = [0.1] * self.num_joints                        # 0 to 1
        self.max_torque = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]                    # 0 to 1
        # self.max_torque = [0.0] * self.num_joints
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
        lcmMotorSub1 = self.lc.subscribe("DXL_STATUS_ARM1",
                                        self.feedback_handler1)
        lcmMotorSub2 = self.lc.subscribe("DXL_STATUS_ARM2",
                                        self.feedback_handler2)
        lcmCoordSub = self.lc.subscribe("COORD_CHANNEL",
                                        self.coordinate_handler)

        lcmMotorSub1.set_queue_capacity(1)
        lcmMotorSub2.set_queue_capacity(1)
        lcmCoordSub.set_queue_capacity(1)

        self.joint_angles = np.copy(self.ready_pose)
        # self.joint_angles = np.copy(self.hit_pose)

        self.cfg_publish_default()
        self.cmd_publish()

        # ===================================== Project begins ===================================== #
        self.dT = 1/100.0
        self.y_offset = 1400
        """ Arm Lengths """
        self.base_len     = 200.0 #113.0
        self.shoulder_len = 98.98
        self.elbow_len    = 98.46
        self.wrist_len    = 350.0 #160.00 #98.0# 41.0

        """ DH Table """
        # radians
        self.dh_table = [{"d" : self.base_len, "a" : 0,                 "alpha": PI/2}, \
                         {"d" : 0,             "a" : self.shoulder_len, "alpha": 0   }, \
                         {"d" : 0,             "a" : self.elbow_len,    "alpha": 0   }, \
                         {"d" : 0,             "a" : self.wrist_len,    "alpha": 0   }]
        # self.last_time = 0
        self.coordinate_list = []

        # Trajectory prediction
        self.mode = 0
        self.trajectory_set = []
        self.vy_set = []
        self.vz_set = []
        self.vy_fit_set = []
        self.vz_fit_set = []
        # Calibration
        self.calibration_pose = None
        # ====================================== Project ends ====================================== #
        
    # def in_range(self, target_pose):
    #     if not (target_pose[0] >= -450 and target_pose[0] <= -330):
    #         return False

    #     if not (target_pose[2] >= 150 and target_pose[2] <= 400):
    #         return False

    #     return True



    def cmd_publish(self):
        """
        Publish the commands to the arm using LCM.
        This publishes the joint angles currently set in the Rexarm object
        You need to call this function to command the arm.
        """
        self.clamp()

        msg = dynamixel_command_list_t()
        msg.len = 2
        
        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            cmd.speed = self.speed_multiplier * self.speed[i]
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
            # print "self.speed_multiplier: ", self.speed_multiplier, " self.torque_multiplier: ", self.torque_multiplier
        self.lc.publish("DXL_COMMAND_ARM1",msg.encode())

        msg = dynamixel_command_list_t()
        msg.len = 4

        for i in range(2, 6):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            cmd.speed = self.speed_multiplier * self.speed[i]
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
            # print "self.speed_multiplier: ", self.speed_multiplier, " self.torque_multiplier: ", self.torque_multiplier
        self.lc.publish("DXL_COMMAND_ARM2",msg.encode())

    def cfg_publish_default(self):
        """
        Publish default configuration to arm using LCM.
        """
        msg = dynamixel_config_list_t()
        msg.len = 2
        for i in range(msg.len):
            cfg = dynamixel_config_t()
            cfg.utime = int(time.time() * 1e6)
            cfg.kp = 250
            if i > 0 and i < 4:
                cfg.ki = 20
            else:
                cfg.ki = 0
            cfg.kd = 0
            cfg.compl_margin = 0
            cfg.compl_slope = 16
            cfg.LED = 1
            msg.configs.append(cfg)
        self.lc.publish("DXL_CONFIG_ARM1",msg.encode())

        msg = dynamixel_config_list_t()
        msg.len = 4
        for i in range(2, 6):
            cfg = dynamixel_config_t()
            cfg.utime = int(time.time() * 1e6)
            cfg.kp = 250
            if i > 0 and i < 4:
                cfg.ki = 20
            else:
                cfg.ki = 0
            cfg.kd = 0
            cfg.compl_margin = 0
            cfg.compl_slope = 16
            cfg.LED = 1
            msg.configs.append(cfg)
        self.lc.publish("DXL_CONFIG_ARM2",msg.encode())

    def cfg_publish(self):
        """
        TODO: implement this function (optional)

        Publish configuration to arm using LCM.
        You need to activelly call this function to command the arm.
        """
        pass

    def get_feedback(self):
        """
        LCM Handler function
        Called continuously with timer by control_station.py
        times out after 10ms
        """
        self.lc.handle_timeout(10)

    def feedback_handler1(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = dynamixel_status_list_t.decode(data)

        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians
            self.speed_fb[i] = msg.statuses[i].speed
            self.load_fb[i] = msg.statuses[i].load
            self.temp_fb[i] = msg.statuses[i].temperature

        # print "We got 1"

    def feedback_handler2(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = dynamixel_status_list_t.decode(data)

        for i in range(msg.len):
            self.joint_angles_fb[i+2] = msg.statuses[i].position_radians
            self.speed_fb[i+2] = msg.statuses[i].speed
            self.load_fb[i+2] = msg.statuses[i].load
            self.temp_fb[i+2] = msg.statuses[i].temperature

        # print "We got 2"

        # print "-----------------------------"
        # print 'id: ', self.id, " time interval Hz: ", 1000000.0/(msg.statuses[-1].utime - self.last_time)
        # self.last_time = msg.statuses[-1].utime

    def coordinate_handler(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = camera_coord_t.decode(data)
        # print "  msg.mode: ", msg.mode
        # if self.mode == 2:
        #     print "self.mode: ", self.mode, "  msg.mode: ", msg.mode
        if self.mode == 2 and msg.mode == 3:
            # print "NO!! must be here for saving!!"
            np.save('trajectory.npy', self.trajectory_set)
            np.save('velocities.npy', [self.vy_set,self.vz_set,self.vy_fit_set,self.vz_fit_set])

        if self.mode == 4:
            if msg.mode == 0 or msg.mode == 1:
                self.mode = msg.mode
            else:
                return
            
        self.mode = msg.mode

        if msg.mode == 0 or msg.mode == 1 or msg.mode == 3:
            self.coordinate_list = []
        elif msg.mode == 2:
            if msg.v_y > 0:
                self.coordinate_list.append(msg)
            else:
                print "msg.v_y <= 0"
        
        elif msg.mode == -1:
            self.calibration_pose = [msg.x, msg.y + 17, msg.z]

    def clamp(self):
        """
        TODO: implement this function

        Clamp Function
        Limit the commanded joint angles to ones physically possible
        so the arm is not damaged.
        """
        self.joint_angles = np.minimum(self.joint_angles, self.upper_angles)
        self.joint_angles = np.maximum(self.joint_angles, self.lower_angles)

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

        return A

    def batter_FK(self, angles, link=3, cfg=1):
        """

        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the
        desired link
        """
        # point of endpoint
        point = [[0], [0], [0], [1]]

        # iterate in reverse and accumulate multiply with point
        for i in xrange(link, -1, -1):
            theta = self.joint_angles_fb[i]
            if i == 0:
                theta = theta + PI/2
            if i == 1:
                theta = theta + PI/2;

            A = self.calc_A_FK(theta, i);
            point = np.matmul(A, point)

        '''
        Calculate phi, phi = 0 when the hand pointing horizontally outward
        phi = -90 deg when the hand is pointing downward
        phi = 90 deg when the hand is pointing upward
        '''
        if cfg == 1:
            if self.joint_angles_fb[2] >= 0:
                phi = -sum(self.joint_angles_fb[1:]) + PI/2
            else:
                phi = sum(self.joint_angles_fb[1:]) + PI/2
        else:
            if self.joint_angles_fb[2] >= 0:
                phi = sum(self.joint_angles_fb[1:]) + PI/2
            else:
                phi = -sum(self.joint_angles_fb[1:]) + PI/2

        phi = clamp_radians(phi)
        return (point[0][0], point[1][0], point[2][0], phi)


    def batter_IK(self, pose):
        """

        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z) which describes the desired
        end effector position and orientation.
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """

        # Figure out where the arm parameters are.
        # How does the frame of reference work?

        # Assuming x, y perpendicular to the table for now
        # print pose

        angles = None

        h = pose[2]
        R = math.sqrt(pose[0]**2 + pose[1]**2)

        d4_offset = 20
        d1 = self.base_len
        d2 = self.shoulder_len
        d3 = self.elbow_len
        d4 = self.wrist_len - d4_offset


        # print "phi = 0" # Fixed typo and errors
        while d4 - 5 > d4_offset:
            try:
                d4 = d4 - 5
                M_squared = (R - d4)**2 + (d1 - h)**2
                alpha = math.atan2(h - d1, R - d4)

                beta_top = -1 * (d3**2) + d2**2 + M_squared
                beta_bottom = 2 * d2 * math.sqrt(M_squared) # beta_bottom = 2 * d3 * math.sqrt(M)
                beta = math.acos(beta_top / beta_bottom)

                gamma_top = -1 * M_squared + d3**2 + d2**2
                gamma_bottom = 2 * d2 * d3
                gamma = math.acos(gamma_top / gamma_bottom)

                theta1 = clamp_radians(math.atan2(pose[1], pose[0]) + PI/2) # theta1 = math.atan2(pose[0], pose[1])
                theta2 = clamp_radians((PI / 2) - alpha - beta)
                theta3 = PI - gamma
                theta4 = clamp_radians(PI/2 - theta2 - theta3)

                # TODO revise this
                angles = [theta1, theta2, theta3, theta4]
            except:
                angles = None
                continue
            if IK_is_clamped(angles):
                print "angle is clamped"
                continue
            if self.batter_has_collision(angles):
                print "has collision"
                continue

            angles = [theta1, theta2, theta3, theta4]
            break

        if angles != None:
            return angles

        print "long reach!"
        d4 = self.wrist_len - d4_offset
        while d4 - 5 > d4_offset:
            try:
                d4 = d4 - 5
                M_squared = R**2 + (d1 - h)**2
                # print "M: ", math.sqrt(M_squared), " sum: ", d2+d3+d4
                alpha = math.atan2(R , d1 - h)

                beta_top = -1 * d4**2 + (d2+d3)**2 + M_squared
                beta_bottom = 2 * (d2+d3) * math.sqrt(M_squared)
                # print "acos arg: ", beta_top / beta_bottom
                beta = math.acos(beta_top / beta_bottom)

                gamma_top = -1 * M_squared + (d2+d3)**2 + d4**2
                gamma_bottom = 2 * (d2+d3) * d4
                gamma = math.acos(gamma_top / gamma_bottom)

                theta1 = clamp_radians(math.atan2(pose[1], pose[0]) + PI/2) # theta1 = math.atan2(pose[0], pose[1])
                theta2 = clamp_radians(PI - alpha - beta)
                theta3 = 0
                theta4 = clamp_radians(PI - gamma)

                # TODO revise this
                angles = [theta1, theta2, theta3, theta4]
            except:
                angles = None
                continue
            if IK_is_clamped(angles):
                print "angle is clamped"
                continue
            if self.batter_has_collision(angles):
                print "has collision"
                continue

            angles = [theta1, theta2, theta3, theta4]
            break

        return angles
            
        
    def batter_has_collision(self, q):
        """

        Perform a collision check with the ground and the base of the Rexarm
        takes a 4-tuple of joint angles q
        returns false if no collision occurs
        """
        
        for link in range(1, 4):
            end_effector = self.batter_FK(angles=q, link = link)
            if link == 1:
                if end_effector[0] > 0:
                    # print "1 collision"
                    return True
            elif link == 2:
                r = np.linalg.norm(end_effector[:2])
                if end_effector[0] > 0 or r < 40:
                    # print "2 collision"
                    return True
            elif link == 3:
                if end_effector[0] > 0:
                    # print "3 collision"
                    return True

        return False

    def trajectory_predict(self):
        if len(self.coordinate_list) < 2:
            print "len(self.coordinate_list): ", len(self.coordinate_list)
            return None, None
        # time_band = 4
        # half_band = time_band/2
        # if len(self.coordinate_list) < time_band:
        #     self.target_pose = None
        #     return

        # coord_list = list(self.coordinate_list[-time_band:-1])
        # vx_list = []
        # vy_list = []
        # vz_list = []
        # for i in range(half_band):
        #     vx_list.append((coord_list[i+half_band].x - coord_list[i].x)/float(coord_list[i+half_band].utime - coord_list[i].utime))
        #     vy_list.append((coord_list[i+half_band].y - coord_list[i].y)/float(coord_list[i+half_band].utime - coord_list[i].utime))
        #     vz_list.append((coord_list[i+half_band].z - coord_list[i].z)/float(coord_list[i+half_band].utime - coord_list[i].utime))

        # vx = np.mean(vx_list)*1e6
        # vy = np.mean(vy_list)*1e6
        # vz = np.mean(vz_list)*1e6

        # revise this
        # init_msg = coord_list[half_band]
        # timer_start = time.time()

        x = [msg.x*1e-3 for msg in self.coordinate_list]

        vy_data = [msg.v_y*1e-3 for msg in self.coordinate_list]
        vz_data = [msg.v_z*1e-3 for msg in self.coordinate_list]
        time_data = [msg.utime/1e6 for msg in self.coordinate_list]
        x_para = np.polyfit(time_data, x, 1)
        # print 'time_data', time_data
        # print 'vy_data', vy_data
        vy_para, _ = curve_fit(fun_vy, time_data, vy_data)
        vz_para, _ = curve_fit(fun_vz, time_data, vz_data)

        x0, y0, z0 = self.coordinate_list[-1].x*1e-3, self.coordinate_list[-1].y*1e-3, self.coordinate_list[-1].z*1e-3
        vx0, vy0, vz0 = x_para[0], fun_vy(time_data[-1], vy_para[0]), fun_vz(time_data[-1], vz_para[0])
        
        self.vy_set.append(vy_data[-1])
        self.vz_set.append(vz_data[-1])
        self.vy_fit_set.append(vy0)
        self.vz_fit_set.append(vz0)

        Y  = [x0, y0, z0, vx0, vy0, vz0]
        t0 = time_data[-1]

        dt = 0.01
        integral = ode(ode_f).set_integrator('dopri5')
        integral.set_initial_value(Y, t0)

        if vy0 > 1: # TODO: Check the condition of vy0 == 0?
            trajectory = []
            while integral.successful() and not (Y[1] > -0.5 and Y[2] < Z_height) and integral.t - t0 < 1:
                print 'y:', Y[1], ',t:', integral.t + dt - t0
                Y = integral.integrate(integral.t + dt)
                trajectory.append(Y[0:3])
            self.trajectory_set.append(trajectory)

            print "get trajectory, len:", len(trajectory)
        
            if len(trajectory) == 0:
                time_left = -1
                target_pose = [x0, y0, z0]
                return target_pose, time_left
            target_pose = Y[0:3] * 1e3
            time_left = integral.t - t0
            if target_pose[1] < 300:
                return target_pose, time_left
            else:
                return None, None
        else:
            print "vy <= 0!!!"
            return None, None

        print "I am done"

    def plan_path(self, target_pose, time_left):
        #TODO revise x coordinate
        if target_pose == None:
            print "self.target_pose is None"
            return

        # if not in_range(target_pose):
        #     print "Not in range!!!-----------"
        #     print 'target_pose: ', target_pose
        #     return

        # Adjust z hit position
        x = -350.0
        while x > -550:
            target_pose[0] = x
            target_pose[2] = Z_height * 1000
            servo_angles = self.batter_IK(target_pose)
            x -= 10
            if servo_angles != None:
                break

        if servo_angles == None:
            return

        if time_left > RESPOND_TIME:
            # print "v_final_min > constant_v"
            return
        # Adjust z hit position
        self.joint_angles[1:4] = servo_angles[1:4]
        self.joint_angles[4] = self.hit_pose[4]
        self.speed[1:5] = [1.0, 1.0, 1.0, 1.0]
        
        
        # Adjust hit speed
        # no time left, hit with maximum speed
        if time_left <= RISE_TIME:
            self.speed[0] = 1.0
            self.joint_angles[0] = self.upper_angles[0]
            self.speed[5] = 1.0
            self.joint_angles[5] = self.upper_angles[5]
            return

        # Adjust base speed
        link_id = 0
        self.adjust_speed(link_id, time_left, servo_angles)
        
        # Adjust bat speed
        # link_id = 5
        # self.adjust_speed(link_id, time_left, )

    def adjust_speed(self, link_id, time_left, servo_angles):
        v_final_max = V_FB2RAD_S * 0.5
        v_final_min = V_FB2RAD_S * 0.0
        v_final_mean = (v_final_max + v_final_min)/2.0

        angle_diff = servo_angles[link_id] - self.joint_angles_fb[link_id]
        delta_t = angle_diff/v_final_mean
        constant_v = angle_diff/(time_left - RISE_TIME)
        self.speed[link_id] = constant_v/V_FB2RAD_S + SPEED_ADDER#* SPEED_FACTOR
        self.joint_angles[link_id] = self.upper_angles[link_id]

    def toReadyPose(self):
        self.speed = [0.1] * self.num_joints
        self.joint_angles = np.copy(self.ready_pose)
        self.cmd_publish()
        self.get_feedback()
        while np.linalg.norm(np.subtract(self.joint_angles_fb, self.ready_pose)) *R2D > 10:
            self.get_feedback()
        # while True:
        #     self.get_feedback()
        #     if np.linalg.norm(np.subtract(self.joint_angles_fb, self.ready_pose)) *R2D < 10:
        #         break

    def moveToBall(self):
        if self.calibration_pose != None:
            pose = self.calibration_pose
            angles = self.batter_IK(pose)
            if angles != None:
                print R2D*np.array(angles)
                self.joint_angles[0:4] = self.batter_IK(pose)
                self.cmd_publish()
            else:
                print 'IK fails!!!'

    def run(self):
        # while True:
        #     self.get_feedback()
        #     if np.linalg.norm(np.subtract(self.joint_angles_fb, self.ready_pose)) *R2D < 10:
        #         break

        self.toReadyPose()
        
        while(True):
            start = time.time()
            """ Code starts"""
            self.get_feedback()
            # print "base", self.joint_angles_fb[0]*R2D, "command: ", self.speed[0], "actual: ", self.speed_fb[0]
            
            # print self.batter_FK(angles=self.joint_angles)
            # self.moveToBall()
            
            if self.mode == 2: # or self.mode == 3:
                print "mode == 2"
                target_pose, time_left = self.trajectory_predict()
                if time_left != None:
                    self.plan_path(target_pose, time_left)
                    self.cmd_publish()
                    if len(self.coordinate_list) > 0:
                        print "-------------------------"
                        msg = self.coordinate_list[-1]
                        print "ball coordinate: ", msg.x, msg.y, msg.z
                        print "forward kinematics: ", self.batter_FK(angles=self.joint_angles_fb)
                        print "base:", self.joint_angles_fb[0]
                        print "bat:", self.joint_angles_fb[5]
                # else:
                #     # print "this is NO and i am here"

            if self.joint_angles_fb[5] > 45 * D2R:
                self.mode = 4
                self.toReadyPose()
                # break


            """ Code ends"""
            elapse = time.time() - start
            if elapse < self.dT:
                time.sleep(self.dT - elapse)
            else:
                print "NO!!! To much computation time each loop!!!"            


def main():
    batter = Batter()
    batter.run()
    

if __name__ == '__main__':
    main()
    
