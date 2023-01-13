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

       
        actions.talk('Starting Take out the Garbage task')
        actions.head("",2.9)
        actions.manip('open')
        actions.goto('bin1')
        aux = False
        actions.talk("Looking for a trash!")
        coordinates = 0
        while not aux:
            coordinates = actions.find_obj('closest')
            if coordinates is not None:
                actions.save_obj_location("lixo_local", coordinates, 0.63)
                actions.talk("Saved!")
                aux = True
            if aux == True:
                print("Salvou local do objeto")
            else :
                print('No object detected')

        actions.talk('Going to the first bin')
        time.sleep(2)
        actions.goto('lixo_local')
        actions.recog_garbage(5,0.25)
        actions.head("",3.14)
        resp = actions.manip('find_trash')
        rospy.sleep(3)
        actions.manip('pick_lixo')
        # if resp == True:
        #    actions.center_filter(0.1, 0.1)
        #    actions.talk('Im at the right point')
        #    actions.manip('pick_lixo')
        # else:
        #    actions.manip('find_trash')
        #    actions.center_filter(0.1, 0.1)
        #    actions.talk('Im at the right point')
        #    actions.manip('pick_lixo')
        # rospy.sleep(3)
        # actions.manip("guardar_lixo")
        # time.sleep(10)
        # actions.manip("home")

        # actions.head("",2.7)
        # actions.talk('Going to the second bin')
        # actions.goto('search_bin2')
        # aux = False
        # actions.talk("Looking for a trash!")
        # coordinates = 0
        # while not aux:
        #     coordinates = actions.find_obj('closest')
        #     if coordinates is not None:
        #         actions.save_obj_location("lixo_local", coordinates, 0.63)
        #         actions.talk("Saved!")
        #         aux = True
        #     if aux == True:
        #         print("Salvou local do objeto")
        #     else :
        #         print('No object detected')

        # actions.talk('Going to the second bin')
        # time.sleep(2)
        # actions.goto('lixo_local')
        # actions.head("",3.14)
        # resp = actions.manip('find_trash')
        # rospy.sleep(3)
        # if resp == True:
        #     actions.center_filter(0.1, 0.1)
        #     actions.talk('Im at the right point')
        #     actions.manip('pick_lixo')
        # else:
        #     actions.manip('find_trash')
        #     actions.center_filter(0.1, 0.1)
        #     actions.talk('Im at the right point')
        #     actions.manip('pick_lixo')

        # time.sleep(5)
        # actions.manip("attack")
        # actions.talk('Going to the collection zone')
        # actions.goto('collection_zone')
        # actions.manip("find_trash")
        # actions.talk('Dropping the first trash')
        # actions.manip("open")
        # actions.dynamixel_goal(6, 1700)
        # actions.manip("tirar_lixo")
        # rospy.sleep(1)
        # actions.manip("find_trash")
        # rospy.sleep(2)
        # actions.talk('Dropping the second trash')
        # actions.manip("open")
        # actions.dynamixel_goal(6, 1700)
        # actions.manip("home")

        # # actions.talk('Going to collection zone')
        # # actions.goto('collection_zone')
        # # actions.manip("", x=0.3, y=0.0, z=0.28, rx=0.0, ry=0.0, rz=0.0)
        # # time.sleep(1)
        # # actions.talk('Dropping the first trash')
        # # actions.manip("open")
        # # actions.dynamixel_goal(6, 2048)  
        # # actions.manip("home")

        # # actions.talk('Going to the second bin')
        # # actions.goto('bin2')
        # # actions.talk('Collecting the second trash')
        # # actions.manip("pick_lixo")
        # # actions.manip("home")       
        actions.talk('Going to collection zone')
        actions.goto('collection_zone')
        actions.manip("drop_lixo")
        # # actions.dynamixel_goal(1, 1900)
        # # actions.talk('Dropping the first trash')
        actions.manip("open")
        # # actions.dynamixel_goal(6, 1700)
        # # actions.manip("tirar_lixo")
        # # rospy.sleep(1)
        # # actions.manip("drop_lixo")
        # # actions.dynamixel_goal(1, 2100)
        # # actions.talk('Dropping the second trash')
        # # actions.manip("open")
        # # actions.dynamixel_goal(6, 1700)
        # # actions.manip("home")
        actions.talk('Task completed')
                
                
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