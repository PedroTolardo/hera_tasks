#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traceback
from turtle import forward
import rospy
import math
import json
import time

from std_msgs.msg import UInt32
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest


from hera_control.srv import Manip3
from hera_control.msg import Manip3_Goal

from Actions import Actions

class Task():
       
    """docstring for Task"""
    def __init__(self):

        

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'ServingDrinks')
        rospy.set_param('/story/storyline',[
            ])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)
        lang = rospy.get_param('~lang', 'en')   
        actions = Actions(lang)

        ########################################################################
        ########################################################################

        actions.move('foward', seconds=2.0)
        actions.manip("tira_tampa")
        rospy.sleep(1)
        actions.manip("pick_trash")
        actions.manip("pick_lixo")
        actions.manip("close")
        actions.manip("home")
        rospy.sleep(3)
        actions.manip("pick_trash")
        actions.manip("guardar_lixo")
        actions.manip("home")

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
