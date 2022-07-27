#! /usr/bin/python3 

import rospy
import actionlib
import spot_kinova_msgs.msg
from spot_kinova_msgs.srv import UiCmd, UiCmdResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse
from spot_msgs.srv import ListGraph, DownloadGraph, UploadGraph, SetLocalizationFiducial
import spot_msgs.msg
import numpy as np
import pinocchio as se3
import importlib, pkgutil
import threading
import cmd, sys, os
import copy
import time

from simulation import bcolors, ControlSuiteShell
#from real2 import bcolors, make_dir, ControlSuiteShell
       
class UiCmdServer(ControlSuiteShell):
    def __init__(self):
        ControlSuiteShell.__init__(self)        
        
        self.ui_cmd_service = rospy.Service('/ui_cmd_service', UiCmd, self.handle_cmd_string)
        
        print('Constructed UiCmdServer()')
        
        self.control_suite_shell = ControlSuiteShell()
        self.functions = {
            'walk_sim' : self.do_walk,
            'qrpick_sim' : self.do_qrpick,
            'pickup_sim' : self.do_pickup,
            'home_sim' : self.do_home,
            'fold_sim' : self.do_fold,
            'ready_sim' : self.do_ready,
            'reach_sim' : self.do_reach,
            'walk_sim' : self.do_walk,
            'body_sim' : self.do_body,
            'wholebody_sim' : self.do_wholebody
        }
               
    def handle_cmd_string(self, req):
        print("Get cmd string request")
        print('request : ', req.command)
        print('argument: ', req.arg,'\n')
        
        self.functions[req.command](int(req.arg))
        
        return UiCmdResponse(
            success=True,
            message="Cmd Response!!"
        )        
        
if __name__ == '__main__':
#    rospy.init_node('ui_server', anonymous=True)
   
    ui_server = UiCmdServer().cmdloop()
    
#    try:
#        rospy.spin()
#    except KeyboardInterrupt:
#        print("Shutting down")

    
