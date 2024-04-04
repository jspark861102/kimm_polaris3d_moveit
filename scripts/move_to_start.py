#! /usr/bin/python3

import rospy
import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import FollowJointTrajectoryActionResult
import numpy as np
import importlib, pkgutil
import threading
import cmd, sys, os
import copy
import time
import math
from tf.transformations import *
from moveit_commander import MoveGroupCommander, RobotCommander
from actionlib_msgs.msg import GoalStatusArray
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

def tf_from_pose_msg(pose_msg):
    q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    t = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    T_mat = quaternion_matrix(q)
    T_mat[:3, 3] = t

    return T_mat

def tf_to_pose_msg(mat):
    t = mat[:3, 3]
    q = quaternion_from_matrix(mat)

    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]

    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    return pose_msg

def skew_matrix(vector):

    return np.array([[ 0,          -vector[2],  vector[1]], 
                     [ vector[2],           0, -vector[0]], 
                     [-vector[1],   vector[0],         0]])

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('franka_moveit')

        # create moveit group
        rospy.wait_for_message('move_group/status', GoalStatusArray)
        self.robot = RobotCommander()
        self.group_name = self.robot.get_group_names()
        self.group = MoveGroupCommander('panda_arm')
        
        self.vel =0.1
        self.group.set_max_velocity_scaling_factor(self.vel)
        print("set_max_velocity_scaling_factor to 0.1")

        # move_result_sub = rospy.Subscriber('/move_group/status', GoalStatusArray, self.move_group_status_callback)
        move_result_sub = rospy.Subscriber('/effort_joint_trajectory_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, self.move_group_status_callback)        

        # create robotiq commander
        self.action_name = 'ns0/command_robotiq_action'    # Use your action server namespace if necesary
        self.robotiq_client = actionlib.SimpleActionClient(self.action_name, CommandRobotiqGripperAction)   
        self.robotiq_client.wait_for_server()  

        # offset                 
        self.T_joint7_to_finger = translation_matrix([0.0, 0.0, 0.247])
        self.T_ee_to_finger = translation_matrix([0.0, 0.0, 0.14])
        
        self.T_camera_to_link7 = quaternion_matrix([0.0, 0.0, 1.0, 0.0])
        self.T_camera_to_link7[:3,3] = ([0.032, 0.077, -0.162])        
        
        self.T_camera_to_finger = np.matmul(self.T_camera_to_link7, self.T_joint7_to_finger)                
        self.T_finger_to_camera = inverse_matrix(self.T_camera_to_finger)        

        # moveit planning is performed w.r.c. end_effector frame
        self.T_ee_to_camera = np.matmul(self.T_ee_to_finger, self.T_finger_to_camera)             

    ######################################## callback ####################################
    def move_group_status_callback(self, msg):
        # self.move_status = msg.status_list[0].status       
        self.move_status = msg.status.status       
        # print(self.move_status)         
    
    ################################# franka set named target #############################
    def do_ready(self, arg):
        'franka ready'

        self.group.set_named_target('ready')
        self.group.go(wait=False)    

    ###################################### franka motion ##################################
    def do_set_velocity(self, arg):
        'set franka velocity'
        
        self.vel = float(arg)
        if(self.vel <= 0.3):
            self.group.set_max_velocity_scaling_factor(self.vel)
        else:
            print("set velocity under 0.3")
    
    def do_s(self, arg): #available only for asnyc_move
        'franka pose'
        
        # self.current_pose = self.group.get_current_pose().pose
        # self.group.go(self.current_pose, True)

        self.group.stop()

    def do_print(self, arg):
        'franka pose'
        
        self.current_pose = self.group.get_current_pose().pose
        print(self.current_pose)
        print(self.group.get_current_state())                
    
    def do_home(self, arg):
        'franka home'        
        
        self.target_joint = [n*math.pi/180. for n in [0.0, 0.0, 0.0, -90.0, 0.0, 90.0, 0.0]]
        
        if(arg != ""):
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_joint, True)            
                print("sync_move home completed")        
        else: #async_move
            self.group.go(self.target_joint, False)            

        ################################ ereon side ############################
    def do_ereon_recog_pose(self, arg):
        'ereon recognition pose'
        
        # self.target_joint = [n for n in [-0.266, 0.622, -0.069, -1.953, 1.239, 1.355, 2.050]]   
        self.target_joint = [n for n in [-0.266, 0.851, -0.060, -1.829, 1.244, 1.383, 1.969]]           

        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_joint, True)            
                print("sync_move ereon_recog_pose completed")        
        else: #async_move
            self.group.go(self.target_joint, False)            

    def do_save_ereon_pose(self, arg):
        'save ereon pose'        

        self.ereon_pose = rospy.wait_for_message('/kimm_rcnn/stop_sign_pose', Pose, timeout=10)                

        # Because moveit reference frame is EE and cannot changed, T_finger_to_camera is used instead of T_ee_to_camera to compensate robotiq gripper offset
        self.T_rcnn_ereon_from_camera_frame = tf_from_pose_msg(self.ereon_pose)                                 
        self.T_rcnn_ereon = np.matmul(self.T_finger_to_camera, self.T_rcnn_ereon_from_camera_frame)                

        print("ereon_pose")  
        print(self.ereon_pose)  

        print("T_rcnn_ereon")
        print(self.T_rcnn_ereon)

    def do_ereon_goto_marker(self, arg):
        'go to in front of ereon marker'

        self.current_pose = self.group.get_current_pose().pose
        self.T_current_pose = tf_from_pose_msg(self.current_pose)

        self.T_rcnn_ereon_offset = translation_matrix([0.0, +0.175, 0.00])   #w.r.t. ee frame

        self.T_target_pose = np.matmul(self.T_current_pose, self.T_rcnn_ereon)        
        self.T_target_pose = np.matmul(self.T_target_pose, self.T_rcnn_ereon_offset)        

        self.target_pose = tf_to_pose_msg(self.T_target_pose)        

        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_pose, True)            
                print("sync_move ereon_goto_marker completed")        
        else: #async_move
            self.group.go(self.target_pose, False)            

    def do_ereon_goto_tray(self, arg):
        'go upon the tray'

        self.current_pose = self.group.get_current_pose().pose
        
        ## in global frame ##
        # self.target_pose = self.current_pose
        # self.target_pose.position.z = self.current_pose.position.y + 0.1                

        ## in local frame ##
        self.T_current_pose = tf_from_pose_msg(self.current_pose)
        self.T_target_pose = np.matmul(self.T_current_pose, translation_matrix([0.0, 0.0, 0.1]))                                
        self.target_pose = tf_to_pose_msg(self.T_target_pose)        
        
        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_pose, True)            
                print("sync_move ereon_goto_tray completed")        
        else: #async_move
            self.group.go(self.target_pose, False)            


    def do_ereon_place_cup(self, arg):
        'place the cup to the tray'

        self.current_pose = self.group.get_current_pose().pose
        
        ## in global frame ##
        # self.target_pose = self.current_pose
        # self.target_pose.position.z = self.current_pose.position.y - 0.08                

        ## in local frame ##
        self.T_current_pose = tf_from_pose_msg(self.current_pose)
        self.T_target_pose = np.matmul(self.T_current_pose, translation_matrix([0.0, -0.08, 0.0]))                                
        self.target_pose = tf_to_pose_msg(self.T_target_pose)        
        
        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_pose, True)            
                print("sync_move ereon_place_cup completed")        
        else: #async_move
            self.group.go(self.target_pose, False)            

    def do_ereon_back_from_tray(self, arg):
        'back from the tray to avoid collision with ereon'

        self.current_pose = self.group.get_current_pose().pose
        
        ## in global frame ##
        # self.target_pose = self.current_pose
        # self.target_pose.position.z = self.current_pose.position.y - 0.3                

        ## in local frame ##
        self.T_current_pose = tf_from_pose_msg(self.current_pose)
        self.T_target_pose = np.matmul(self.T_current_pose, translation_matrix([0.1, 0.05, -0.3]))                                
        self.target_pose = tf_to_pose_msg(self.T_target_pose)        
        
        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_pose, True)            
                print("sync_move ereon_back_from_tray completed")        
        else: #async_move
            self.group.go(self.target_pose, False)            

        ################################ cup side ############################
    def do_cup_recog_pose(self, arg):
        'cup recognition pose'
        
        # self.target_joint = [n for n in [0.201, 1.106, -0.064, -1.778, -1.383, 1.526, 1.328]]                
        self.target_joint = [n for n in [0.196, 1.245, -0.062, -1.645, -1.377, 1.529, 1.326]]                        

        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_joint, True)            
                print("sync_move cup_recog_pose completed")        
        else: #async_move
            self.group.go(self.target_joint, False)            

    def do_save_cup_pose(self, arg):
        'save cup pose'           

        count = 0
        while(True):
            if(count >= 5):
                return True

            self.cup_pose = rospy.wait_for_message('/kimm_rcnn/cup_pose', Pose, timeout=20)                        

            # Because moveit reference frame is EE and cannot changed, T_finger_to_camera is used instead of T_ee_to_camera to compensate robotiq gripper offset
            self.T_rcnn_cup_from_camera_frame = tf_from_pose_msg(self.cup_pose)                                 

            if(self.T_rcnn_cup_from_camera_frame[2,3] < 0.5 and self.T_rcnn_cup_from_camera_frame[2,3] > 0.0):
                self.T_rcnn_cup = np.matmul(self.T_finger_to_camera, self.T_rcnn_cup_from_camera_frame)                

                print("cup_pose")  
                print(self.cup_pose)  

                print("T_rcnn_cup")
                print(self.T_rcnn_cup) 

                break
            else:
                print("cup_pose is not reasonable!")
                count += 1

    def do_cup_goto_pick_pose(self, arg):
        'go to cup to pick it'

        self.current_pose = self.group.get_current_pose().pose
        self.T_current_pose = tf_from_pose_msg(self.current_pose)        
        
        self.T_rcnn_cup_offset = translation_matrix([0.0, -0.01, 0.05])   #w.r.t. ee frame

        self.T_target_pose = np.matmul(self.T_current_pose, self.T_rcnn_cup)        
        print(self.T_target_pose)
        self.T_target_pose = np.matmul(self.T_target_pose, self.T_rcnn_cup_offset)        
        print(self.T_target_pose)

        self.target_pose = tf_to_pose_msg(self.T_target_pose)                        

        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_pose, True)            
                print("sync_move cup_goto_pick_pose completed")        
        else: #async_move
            self.group.go(self.target_pose, False)         

    def do_cup_pickup(self, arg):
        'pick up the cup'

        self.current_pose = self.group.get_current_pose().pose
        
        ## in global frame ##
        # self.target_pose = self.current_pose
        # self.target_pose.position.z = self.current_pose.position.z + 0.1                

        ## in local frame ##
        self.T_current_pose = tf_from_pose_msg(self.current_pose)
        self.T_target_pose = np.matmul(self.T_current_pose, translation_matrix([0.0, 0.15, 0.0]))                                
        self.target_pose = tf_to_pose_msg(self.T_target_pose)                

        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_pose, True)            
                print("sync_move cup_pickup completed")        
        else: #async_move
            self.group.go(self.target_pose, False) 

    def do_inter_pose(self, arg):
        'inter pose btw cup and ereon'
        
        self.target_joint = [n for n in [0.218, 0.975, -0.205, -1.863, 0.135, 1.262, 1.442]]        

        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.go(self.target_joint, True)            
                print("sync_move inter_pose completed")        
        else: #async_move
            self.group.go(self.target_joint, False)                   

    ################################### cartesian jog ###################################
    def do_up(self, arg):
        '0.5m up the franka in cartesian space'
        
        self.target_pose = self.group.get_current_pose().pose
        self.target_pose.position.z = self.target_pose.position.z + 0.05        
        self.group.go(self.target_pose, False)

    def do_down(self, arg):
        '0.5m down the franka in cartesian space'
        
        self.target_pose = self.group.get_current_pose().pose
        self.target_pose.position.z = self.target_pose.position.z - 0.05        
        self.group.go(self.target_pose, False)

    def do_left(self, arg):
        '0.5m left the franka in cartesian space'
        
        self.target_pose = self.group.get_current_pose().pose
        self.target_pose.position.y = self.target_pose.position.y + 0.05        
        self.group.go(self.target_pose, False)

    def do_right(self, arg):
        '0.5m right the franka in cartesian space'
        
        self.target_pose = self.group.get_current_pose().pose
        self.target_pose.position.y = self.target_pose.position.y - 0.05        
        self.group.go(self.target_pose, False)

    def do_forward(self, arg):
        '0.5m forward the franka in cartesian space'
        
        self.target_pose = self.group.get_current_pose().pose
        self.target_pose.position.x = self.target_pose.position.x + 0.05        
        self.group.go(self.target_pose, False)

    def do_backward(self, arg):
        '0.5m backward the franka in cartesian space'
        
        self.target_pose = self.group.get_current_pose().pose
        self.target_pose.position.x = self.target_pose.position.x - 0.05        
        self.group.go(self.target_pose, False)
                        
    ################################### robotiq gripper ###################################
    def do_open(self, arg):
        'robotiq open'

        Robotiq.open(self.robotiq_client, block=False)   # Open and do not block thread while completing goal
        print("gripper open")

    def do_open_for_cup_place(self, arg):
        'robotiq position 75mm' 

        Robotiq.goto(self.robotiq_client, pos=0.075, speed=0.01, force=10, block=False)  # Send command Pose[m], speed[m/s], force [%]
        print("gripper open for cup place")

    def do_close(self, arg):
        'robotiq close'

        Robotiq.close(self.robotiq_client, block=True)   # Close and wait until completion
        print("gripper close")

    def do_close_to_cup_position(self, arg):
        'robotiq position 70mm' 

        Robotiq.goto(self.robotiq_client, pos=0.070, speed=0.01, force=10, block=False)  # Send command Pose[m], speed[m/s], force [%]
        print("gripper close to cup position")

    def do_emergency(self, arg):
        'robotiq emergency'

        Robotiq.emergency_release(self.robotiq_client)   # Slowly open gripper and deactivate it

    ######################################## sequences ####################################                        
    def do_test_kimm(self, arg):  
        'test pick and place sequence'
        
        self.do_set_velocity(0.3)

        is_sync = 1 #use arg as 1 to know completeness with sync_move
        
        self.do_open(arg)        
        self.do_cup_recog_pose(is_sync)        
        
        time.sleep(3)                      #wait for nonblurred image
        self.do_save_cup_pose(arg)
        self.do_cup_goto_pick_pose(is_sync)
        self.do_close_to_cup_position(arg)
        self.do_cup_pickup(is_sync)        

        self.do_ereon_recog_pose_from_pickup(is_sync)
        
        time.sleep(3)                      #wait for nonblurred image
        self.do_save_ereon_pose(arg)
        self.do_ereon_goto_marker(is_sync)
        self.do_ereon_goto_tray(is_sync)
        
        self.do_open_for_cup_place(arg)
        self.do_ereon_place_cup(is_sync) #simultaneous motion with gripper open        
        self.do_open(arg)        
        self.do_ereon_back_from_tray(is_sync)
        self.do_cup_recog_pose(is_sync)        

        self.do_set_velocity(0.1)
        print("completed")   

    def do_test(self, arg):  
        'test pick and place sequence'
        
        self.do_set_velocity(0.3)

        is_sync = 1 #use arg as 1 to know completeness with sync_move
        
        self.do_open(arg)        
        self.do_cup_recog_pose(is_sync)        
        
        time.sleep(3)                      #wait for nonblurred image
        self.do_save_cup_pose(arg)
        # self.do_cup_goto_pick_pose(is_sync)
        self.do_cup_goto_pick_pose_with_interpose(is_sync)
        self.do_close_to_cup_position(arg)
        self.do_cup_pickup(is_sync)        

        self.do_ereon_recog_pose_from_pickup(is_sync)
        
        time.sleep(3)                      #wait for nonblurred image
        self.do_save_ereon_pose(arg)
        self.do_ereon_goto_marker(is_sync)
        self.do_ereon_goto_tray(is_sync)
        
        self.do_open_for_cup_place(arg)
        self.do_ereon_place_cup(is_sync) #simultaneous motion with gripper open        
        self.do_open(arg)        
        self.do_ereon_back_from_tray(is_sync)
        self.do_cup_recog_pose(is_sync)        

        self.do_set_velocity(0.1)
        print("completed")  

    ######################################## waypoints ####################################        
    def do_ereon_recog_pose_from_pickup(self, arg):
        'go to ereon recog pose from pickup pose with interpolation'
        
        # Set waypoints
        waypoints = []
        
        wpose = Pose()

        # inter_pose
        wpose.position.x = 0.501
        wpose.position.y = 0.004
        wpose.position.z = 0.09
        wpose.orientation.x = 0.499
        wpose.orientation.y = -0.516
        wpose.orientation.z = -0.498
        wpose.orientation.w =  0.487

        waypoints.append(copy.deepcopy(wpose))

        #ereon_recog_pose
        wpose.position.x =  0.538
        wpose.position.y = -0.058
        wpose.position.z =  0.091
        wpose.orientation.x =  0.040
        wpose.orientation.y = -0.709
        wpose.orientation.z = -0.704
        wpose.orientation.w =  0.015 
        
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)  # 0.01은 step_size, 0.0은 jump_threshold        
        replan= self.group.retime_trajectory(self.robot.get_current_state(), plan, self.vel) #parameter that changes velocity            
        
        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.execute(replan, wait=True) 
                print("sync_move ereon_recog_pose_from_pickup completed")        
        else: #async_move
            self.group.execute(replan, wait=False)                 

    def do_cup_goto_pick_pose_with_interpose(self, arg):
        'go to cup to pick it'
        
        # Set waypoints
        waypoints = []
        
        wpose = Pose()

        self.current_pose = self.group.get_current_pose().pose
        self.T_current_pose = tf_from_pose_msg(self.current_pose)           

        # inter_pose
        self.inter_pose = self.current_pose
        self.inter_pose.position.z += 0.05        
        self.inter_pose.position.y -= 0.05        
        waypoints.append(copy.deepcopy(self.inter_pose))

        # target_pose        
        self.T_rcnn_cup_offset = translation_matrix([0.0, -0.01, 0.05])   #w.r.t. ee frame

        self.T_target_pose = np.matmul(self.T_current_pose, self.T_rcnn_cup)                
        self.T_target_pose = np.matmul(self.T_target_pose, self.T_rcnn_cup_offset)        
        print(self.T_target_pose)

        self.target_pose = tf_to_pose_msg(self.T_target_pose)                        
        waypoints.append(copy.deepcopy(self.target_pose))

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)  # 0.01은 step_size, 0.0은 jump_threshold        
        replan= self.group.retime_trajectory(self.robot.get_current_state(), plan, 0.1) #parameter that changes velocity            
        
        if(arg != ""): 
            if(int(arg) == 1): #sync_move
                self.group.execute(replan, wait=True) 
                print("sync_move ereon_recog_pose_from_pickup completed")        
        else: #async_move
            self.group.execute(replan, wait=False)                 

    ########################################## quit #######################################
    def do_quit(self, arg):
        'quit'
        return True   

if __name__ == '__main__':       
    ControlSuiteShell().cmdloop()




