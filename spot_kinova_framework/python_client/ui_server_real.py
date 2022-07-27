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
#import pinocchio as se3
import importlib, pkgutil
import threading
import cmd, sys, os
import copy
import time

#from simulation import bcolors, ControlSuiteShell
from real7 import bcolors, make_dir, ControlSuiteShell
       
class UiCmdServer(ControlSuiteShell):
    def __init__(self):
        ControlSuiteShell.__init__(self)
        self.ui_cmd_service = rospy.Service('/ui_cmd_service', UiCmd, self.handle_cmd_string)
        
        print('Constructed UiCmdServer()')
        
        self.control_suite_shell = ControlSuiteShell()
        self.functions = {
            'mechcall' : self.do_mechcall,
            'qr1save' : self.do_qr1save,
            'qr2save' : self.do_qr2save,
            'qr3save' : self.do_qr3save,
            'qr4save' : self.do_qr4save,
            'qr5save' : self.do_qr5save,
            'arm_mechpose' : self.do_arm_mechpose,
            'mechqrpick' : self.do_mechqrpick,
            'position' : self.do_position,
            'qrpick' : self.do_qrpick,
            'pickup' : self.do_pickup,
            'gotobox' : self.do_gotobox,
            #'goto_qrbox' : self.do_goto_qrbox,
            'qrboxpick' : self.do_qrboxpick,
            'preready' : self.do_preready,                        
            'prehome' : self.do_prehome,
            'prefold' : self.do_prefold,
            'home' : self.do_home,
            'open' : self.do_open,
            'close' : self.do_close,
            'walk_onestep' : self.do_walk_onestep,
            'start_recording' : self.do_start_recording,
            'stop_recording' : self.do_stop_recording,
            'clear_graph' : self.do_clear_graph,
            'download_graph' : self.do_download_graph,
            'upload_graph' : self.do_upload_graph,
            'list_graph' : self.do_list_graph,
            'set_localization_fiducial' : self.do_set_localization_fiducial,
            'navigate_to' : self.do_navigate_to,
            'test_mech_pnp' : self.do_test_mech_pnp,
            'test_box_pick' : self.do_test_box_pick,
            
            #TODO : arm
            'armup' : self.do_armup,
            'armdown' : self.do_armdown,
            'armleft' : self.do_armleft,
            'armright' : self.do_armright,
            'armforwad' : self.do_armforward,
            'armback' : self.do_armback,         
            'reach' : self.do_reach,

            #'body_sim' : self.do_body,
            'wholebody_sim' : self.do_wholebody,

            #New Navigation
            'save_goals' : self.do_save_goals,
            'load_goals' : self.do_load_goals
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

    
