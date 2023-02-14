#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traceback
import rospy
import math
import json
import time

from std_msgs.msg import UInt32
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest

from Actions import Actions

rule_forbi = "You are breaking the forbidden room rule. You can't be here. Please follow me to the next room to fix it."
rule_drink = "You are breaking the compulsory hidatrion rule. Please follow me to the kitchen so i can give you a drink."
rule_trash = "You are breaking the no literring rule. Please take the garbage and and follow me."
rule_shoes = "You are breaking the no shoes rule. Please follow me to the entrance door to take out your shoes. "


class Task():
       
    """docstring for Task"""
    def __init__(self):        
        rospy.sleep(1)
        lang = rospy.get_param('~lang', 'en')

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'SticklerForTheRules')
        rospy.set_param('/story/storyline',[
            ])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        self.actions = Actions(lang)

        # task variables
        specs = ['yes', 'no']

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = specs

        ########################################################################
        ########################################################################         

        # self.actions.talk('Rule number 1: Forbidden room')
        self.actions.talk('Checking forbidden room')
        self.actions.goto('forbidden_room')
        self.actions.talk('Looking for guests')


        #if(self.found_person()):
        self.guide_person('room2', rule_forbi)

        self.actions.talk ('Going back to the forbidden room')
        self.actions.goto('forbidden_room')
        self.actions.talk('I dont see anyone here breaking the rule.')


    def found_person(self):
        coordinates = None
        #for i in range(10):
        #    coordinates = self.actions.find_obj('closest')

        if(coordinates is not None):
            self.actions.save_obj_location("person", coordinates, 1.00)
            return True
        else:
            return False

    def guide_person(self, local, rule):
        self.actions.talk('I found a guest breaking a rule')
        self.actions.talk('Im going to that position')
        self.actions.goto('person')
        self.actions.talk('Hello guest.')
        self.actions.talk(rule)
        self.actions.goto(local)


if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
