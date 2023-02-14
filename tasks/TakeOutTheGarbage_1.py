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

class Task():
       
    """docstring for Task"""
    def __init__(self):        
        rospy.sleep(1)
        lang = rospy.get_param('~lang', 'en')

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'TakeOutTheGarbage')
        rospy.set_param('/story/storyline',[
            ])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        actions = Actions(lang)

        # task variables
        robotnames = ['Robot','Hera','Ivy']
        locals = ['bin1','bin2','collection_zone']
        specs = [
            '<robotnames>',
            'Start task'
        ]

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = specs
        self.srv.choices.append(Opcs(id="robotnames", values=robotnames))
        self.srv.choices.append(Opcs(id="locals", values=locals))

        ########################################################################
        ########################################################################

       
        actions.head("reset",0.0)
        actions.manip('open')
        actions.manip('home')
        actions.goto('bin1')
        aux = False
        coordinates = 0
        while not aux:
            actions.head("",2.9)
            coordinates = actions.find_obj('closest')
            if coordinates is not None:
                actions.save_obj_location("lixo_local", coordinates, 0.63)
                actions.talk("Saved!")
                aux = True
            if aux == True:
                print("Salvou local do objeto")
            else :
                print('No object detected')
        time.sleep(2)
        actions.goto('lixo_local')
        actions.recog_garbage(5,0.25)
        actions.head("reset",3.14)
        resp = actions.manip('find_trash')
        rospy.sleep(3)
        actions.manip('pick_lixo')   
        actions.talk('Going to collection zone')
        actions.goto('collection_zone')
        actions.manip("drop_lixo")
        actions.manip("open")
                
                
                ######
                # talk = False
                # while not json.loads(actions.question('is_front_free').result):
                #     if not talk:
                #         actions.talk('Waiting to open the door')
                #         talk = True
                #     continue
                # actions.talk('The door is open!')
                # actions.move('foward', seconds=5.0)


if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass