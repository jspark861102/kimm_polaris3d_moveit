#! /usr/bin/python3
import rospy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int16
import cmd, sys, os

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
    prompt = "(estop) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('moveit_estop')       
        self.estop_pub = rospy.Publisher('/effort_joint_trajectory_controller/follow_joint_trajectory/cancel', GoalID, queue_size=1)        
        # self.estop_pub = rospy.Publisher('/execute_trajectory/cancel', GoalID, queue_size=1)                

    def do_s(self, arg):        
        cancel_msg = GoalID()
        cancel_msg.stamp = rospy.Time.now()
        
        self.estop_pub.publish(cancel_msg)

    def do_quit(self, arg):
        'quit'
        return True          

if __name__ == '__main__':       
    ControlSuiteShell().cmdloop()







