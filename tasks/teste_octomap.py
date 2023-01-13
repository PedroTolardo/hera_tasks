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

        # task variables
        robotnames = ['Robot','Hera','Ivy']
        locals = ['party','bar','pessoa1','pessoa2','pessoa3']
        specs = [
            '<robotnames>',
            'Start task'
        ]

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = specs
        self.srv.choices.append(Opcs(id="robotnames", values=robotnames))
        self.srv.choices.append(Opcs(id="locals", values=locals))

        srv_names = StartRequest()
        srv_names.spec = [
            'Hunter',
            'William', 'Wagner', 'Caio']

        srv_drinks = StartRequest()
        srv_drinks.spec = [
            'Lemonade',
            'Water', 'Coke']

        srv_answer = StartRequest()
        srv_answer.spec = ['yes','no']

        ########################################################################
        ########################################################################

        aux = False
#        actions.goto('start')
#        actions.talk('Starting task')
#        actions.manip("open")
#        actions.manip("home")
#        actions.head("",3.0)
#
#        actions.goto('shelf')
#        actions.move('foward', seconds=1.5)
        actions.head("",3.0)
        while not aux:
            coordinates = actions.find_obj('closest')
            print("coordenda: ",coordinates)
            if coordinates is not None:
    
                success = actions.manip_goal(coordinates, 'pick2')
            
                if success == "success":
                    actions.talk("pick executado")
            #    actions.head("reset",0.0)
                    success = actions.manip_goal(coordinates, 'place2')
                    if success:
                        actions.talk("place executado")
                        aux = True

            #    aux = True
            #else :
            #    actions.talk("nao rolou pick")
            #    actions.manip("home")
#        
#        actions.goto('place')
        #actions.head("",2.5)

        #coordinates.position.x = 0.424
        #coordinates.position.y = 0.01
        #coordinates.position.z = 0.1
        #coordinates.position.rx = 0.0
        #coordinates.position.ry = 0.0
        #coordinates.position.rz = 0.024
        #actions.manip_goal(coordinates, 'place')

        #actions.goto('start')
        #actions.manip("open")
        #actions.manip("home")
        #actions.talk('Ending task')

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
