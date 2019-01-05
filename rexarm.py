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

"""Useful Conversions"""
PI = np.pi
D2R = PI/180.0
ANGLE_TOL = 2*PI/180.0

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
    JA = np.minimum(np.asarray(IK_joint_angle), [PI, 120 * D2R])#, 120 * D2R, 120 * D2R])
    JA = np.maximum(JA, [-1 * PI, -120 * D2R])#, -120 * D2R, -120 * D2R])
    if np.array_equal(np.asarray(IK_joint_angle), JA):
        return False
    else:
        return True

""" Rexarm Class """
class Rexarm():
    def __init__(self):

        """ TODO: modify this class to add functionality you need """

        """ Commanded Values """
        self.num_joints = 2                         # number of motors, increase when adding gripper
        self.joint_angles = [0.0] * self.num_joints # radians
        self.move_test_flag = 0

        # you must change this to an array to control each joint speed separately
        self.speed = [100.0, 100.0]#, 100.0, 100.0, 100.0, 100.0]                         # 0 to 1
        self.max_torque = [100.0, 100.0]#, 100.0, 100.0, 100.0, 100.0]                    # 0 to 1
        self.speed = [1.0, 1.0]#, 1.0, 1.0, 1.0, 1.0]                         # 0 to 1
        self.max_torque = [1.0, 1.0]#, 1.0, 1.0, 1.0, 1.0]                    # 0 to 1
        self.speed_multiplier = 1 #0.5
        self.torque_multiplier = 1 #0.5

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
        lcmMotorSub = self.lc.subscribe("DXL_STATUS",
                                        self.feedback_handler)

        # ===================================== Project begins ===================================== #
        """ Arm Lengths """
        self.base_len     = 113.0
        self.shoulder_len = 98.98
        self.elbow_len    = 98.46
        self.wrist_len    = 160.00 #98.0# 41.0

        """ DH Table """
        # radians
        self.dh_table = [{"d" : self.base_len, "a" : 0,                 "alpha": PI/2}, \
                         {"d" : 0,             "a" : self.shoulder_len, "alpha": 0   }, \
                         {"d" : 0,             "a" : self.elbow_len,    "alpha": 0   }, \
                         {"d" : 0,             "a" : self.wrist_len,    "alpha": 0   }]
        self.last_time = 0
        # ====================================== Project ends ====================================== #
        

    def cmd_publish(self):
        """
        Publish the commands to the arm using LCM.
        This publishes the joint angles currently set in the Rexarm object
        You need to call this function to command the arm.
        """
        msg = dynamixel_command_list_t()
        msg.len = self.num_joints
        self.clamp()

        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            cmd.speed = self.speed_multiplier * self.speed[i]
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
            print "self.speed_multiplier: ", self.speed_multiplier, " self.torque_multiplier: ", self.torque_multiplier
        self.lc.publish("DXL_COMMAND",msg.encode())

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
        self.lc.publish("DXL_CONFIG",msg.encode())

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

    def feedback_handler(self, channel, data):
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

        print "-----------------------------"
        print 'id: ', self.id, " time interval Hz: ", 1000000.0/(msg.statuses[-1].utime - self.last_time)
        self.last_time = msg.statuses[-1].utime

    def clamp(self):
        """
        TODO: implement this function

        Clamp Function
        Limit the commanded joint angles to ones physically possible
        so the arm is not damaged.
        """

        self.joint_angles = np.minimum(self.joint_angles, [PI, 120 * D2R])#-, 120 * D2R, 120 * D2R, 180* D2R, 90 * D2R])
        self.joint_angles = np.maximum(self.joint_angles, [-PI, -120 * D2R])#, -120 * D2R, -120 * D2R, -180* D2R, -90 * D2R])

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

    def rexarm_FK(self, link, cfg=1):
        """
        TODO: implement this function

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
        return (-point[0], -point[1], point[2], phi)

    #Sagar said to just not use cfg down and to just do long reach, high reach
    def rexarm_IK(self, pose, cfg=1):
        """
        TODO: implement this function

        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """

        # Figure out where the arm parameters are.
        # How does the frame of reference work?

        # Assuming x, y perpendicular to the table for now
        print pose
        h = pose[2]
        R = math.sqrt(pose[0]**2 + pose[1]**2)
        # d1 = .113
        # d2 = .09898
        # d3 = .09846
        # d4 = .03456

        d1 = self.base_len
        d2 = self.shoulder_len
        d3 = self.elbow_len
        d4 = self.wrist_len

        if not cfg:
            return NONE

        #dummy value for now, need to measure
        try:
            # if R < 190 and h < 180: # phi = -90
            try:
                print "phi = -90" # Fixed typo and errors
                # Normal Reach. d1 and d4 are both perpendicular
                M_squared = R**2 + (d4 + h - d1)**2 # M_squared = R**2 + (d4 + h + d1)**2
                alpha = math.atan2(d4 + h - d1, R)

                beta_top = -1 * (d3**2) + d2**2 + M_squared
                beta_bottom = 2 * d2 * math.sqrt(M_squared) # beta_bottom = 2 * d2 * math.sqrt(M)
                beta = math.acos(beta_top / beta_bottom)

                gamma_top = -1 * M_squared + d2**2 + d3**2
                gamma_bottom = 2 * d2 * d3
                gamma = math.acos(gamma_top / gamma_bottom)

                theta1 = clamp_radians(math.atan2(pose[1], pose[0]) - PI/2) # theta1 = math.atan2(pose[0], pose[1])
                theta2 = clamp_radians((PI / 2) - alpha - beta)
                theta3 = PI - gamma
                theta4 = clamp_radians(PI - theta2 - theta3)

                angles = (theta1, theta2, theta3, theta4)
                if IK_is_clamped(angles):
                    raise Exception('Angle clamped!')

            except: # phi = 0
                print "phi = 0" # Fixed typo and errors
                M_squared = (R - d4)**2 + (d1 - h)**2
                alpha = math.atan2(h - d1, R - d4)

                beta_top = -1 * (d3**2) + d2**2 + M_squared
                beta_bottom = 2 * d2 * math.sqrt(M_squared) # beta_bottom = 2 * d3 * math.sqrt(M)
                beta = math.acos(beta_top / beta_bottom)

                gamma_top = -1 * M_squared + d3**2 + d2**2
                gamma_bottom = 2 * d2 * d3
                gamma = math.acos(gamma_top / gamma_bottom)

                theta1 = clamp_radians(math.atan2(pose[1], pose[0]) - PI/2) # theta1 = math.atan2(pose[0], pose[1])
                theta2 = clamp_radians((PI / 2) - alpha - beta)
                theta3 = PI - gamma
                theta4 = clamp_radians(PI/2 - theta2 - theta3)

                angles = (theta1, theta2, theta3, theta4)
                if IK_is_clamped(angles):
                    raise Exception('Angle clamped!')

        except:
            try:
                print "long reach" # Fixed typo and errors
                M_squared = R**2 + (d1 - h)**2
                print "M: ", math.sqrt(M_squared), " sum: ", d2+d3+d4
                alpha = math.atan2(R , d1 - h)

                beta_top = -1 * d4**2 + (d2+d3)**2 + M_squared
                beta_bottom = 2 * (d2+d3) * math.sqrt(M_squared)
                print "acos arg: ", beta_top / beta_bottom
                beta = math.acos(beta_top / beta_bottom)

                gamma_top = -1 * M_squared + (d2+d3)**2 + d4**2
                gamma_bottom = 2 * (d2+d3) * d4
                gamma = math.acos(gamma_top / gamma_bottom)

                theta1 = clamp_radians(math.atan2(pose[1], pose[0]) - PI/2) # theta1 = math.atan2(pose[0], pose[1])
                theta2 = clamp_radians(PI - alpha - beta)
                theta3 = 0
                theta4 = clamp_radians(PI - gamma)

                angles = (theta1, theta2, theta3, theta4)
                if IK_is_clamped(angles):
                    raise Exception('Angle clamped!')
            except:
                # print "exception raised - returning NONE"
                # return None
                raise Exception('Angle clamped!')

        angles = (theta1, theta2, theta3, theta4)
        if IK_is_clamped(angles):
            # print "IK is clamped triggered - returning NONE"
            # return None
            raise Exception('Angle clamped!')

        return (theta1, theta2, theta3, theta4)
            
        
    def rexarm_collision_check(self, q):
        """
        TODO: implement this function

        Perform a collision check with the ground and the base of the Rexarm
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        
        last_link = 3

        # point of endpoint
        point = [[0], [0], [0], [1]]

        # iterate in reverse and accumulate multiply with point
        for i in xrange(last_link, -1, -1):
            theta = q[i]
            if i == 1:
                theta = theta + PI/2;

            A = self.calc_A_FK(theta, i);
            point = np.matmul(A, point)

        Z = point[2]

        if Z > 0:
            return true
        else:
            return false
    
    def plan_path(self,target_pose,mode = 0):

        ## mode for further optimization
        ## first go straight
        ## then go to target
        # mode = 0 moving home
        # mode = 1 pick
        # mode = 2 drop
        # mode = 3 move and hold
        # mode = 4 direct move and drop
        
        gripper_release = 30
        gripper_hold = -12

        path = []
        velocity = []
        torque = []
        
        step = 0
        speed_v = [30,30,30,30,30]
        speed_v_slow = [10,10,10,10,10]
        torque_v = [100,100,100,100,100]

        current_pose = self.joint_angles_fb
        ori_base = current_pose[0]
        ori_shoulder =  current_pose[1]
        ori_elbow = current_pose[2]
        ori_wrist = current_pose[3]
                # 1. base  2. wrist 3. elbow 4. sholder 5. gripper
        

        if mode == 0: # moving home
            ori_gripper = 30 * D2R
            target_gripper = 30 * D2R
            target_base,target_shoulder,target_elbow,target_wrist = 0, 0, 0, 0

            ## return home 
            ## 1.gripper 2. wrist 3.elbow 4.shoulder 5.base
            size = (self.num_joints, self.num_joints)
            path = np.zeros(size)
            velocity = np.zeros(size)
            torque = np.zeros(size)

            path[step] = [ori_base,ori_shoulder,ori_elbow,ori_wrist,target_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v
            step = step + 1

            path[step] = [ori_base,target_shoulder,ori_elbow,ori_wrist,target_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v
            step = step + 1

            path[step] = [ori_base,target_shoulder,target_elbow,ori_wrist,target_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v
            step = step + 1

            path[step] = [ori_base,target_shoulder,target_elbow,target_wrist,target_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v
            step = step + 1
            
            path[step] = [target_base,target_shoulder,target_elbow,target_wrist,target_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v
            step = step + 1
        
        else:
            target_base,target_shoulder,target_elbow,target_wrist = self.rexarm_IK(target_pose)

            if mode == 1:
                ori_gripper = gripper_release * D2R
                target_gripper = gripper_hold * D2R
            elif mode == 2:
                ori_gripper = gripper_hold * D2R
                target_gripper = gripper_release* D2R
            elif mode == 3:
                ori_gripper = self.joint_angles_fb[4]
                target_gripper = ori_gripper
                speed_v = [30,30,30,30,30]
            elif mode == 4:
                ori_gripper = self.joint_angles_fb[4]
                target_gripper = ori_gripper
                speed_v = [5,5,5,5,5]

            error = np.linalg.norm(np.array(current_pose) - np.array([0.0] * self.num_joints)) ##difference in base

            if (error > 3*D2R and mode != 4): ## go back to base first
                ## back to origin 1 shoulder 2. elbow 
                size = (self.num_joints + 2.0,self.num_joints)
                path = np.zeros(size)
                velocity = np.zeros(size)
                torque = np.zeros(size)

                ## 1. adjust shoulder
                path[step] = [ori_base,0.0,ori_elbow,ori_wrist, ori_gripper]
                velocity[step] = speed_v
                torque[step] = torque_v
                ori_shoulder = 0.0

                step = step + 1

                ## 2. adjust elbow
                path[step] = [ori_base,0.0,0.0,ori_wrist, ori_gripper]
                velocity[step] = speed_v
                torque[step] = torque_v
                ori_elbow = 0.0

                step = step + 1
            else:   
                size = (self.num_joints,self.num_joints)
                path = np.zeros(size)
                velocity = np.zeros(size)
                torque = np.zeros(size)            
        
            ## 1. adjust base
            path[step] = [target_base,ori_shoulder,ori_elbow,ori_wrist,ori_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v
            step = step + 1

            ## 2. adjust wrist
            path[step] = [target_base,ori_shoulder,ori_elbow,target_wrist,ori_gripper]
            if mode == 2:
                velocity[step] = speed_v_slow
            else:
                velocity[step] = speed_v
            torque[step] = torque_v

            step = step + 1

            ## 3. adjust elbow
            path[step] = [target_base,ori_shoulder,target_elbow,target_wrist,ori_gripper]
            velocity[step] = speed_v
            torque[step] = torque_v

            step = step + 1

            ## 4. adjust wrist
            #print step
            path[step] = [target_base,target_shoulder,target_elbow,target_wrist,ori_gripper]
            velocity[step] = speed_v_slow
            torque[step] = torque_v

            step = step + 1

            ## 5. grab
            #print step
            path[step] = [target_base,target_shoulder,target_elbow,target_wrist,target_gripper]
            velocity[step] = speed_v_slow
            torque[step] = torque_v

        return path,velocity,torque
    
    def interpolate(self,ori_paths):

        speed_v = [20.0,20.0,20.0,20.0,20.0]
        torque_v = [25.0,25.0,25.0,25.0,25.0]

        num_points = len(ori_paths)

        size = (num_points,5)

        velocity = np.zeros(size)
        torque = np.zeros(size)

        for i in range(num_points):
            velocity[i] = speed_v
            torque[i] = torque_v

        return ori_paths,velocity,torque
