#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traceback
import rospy
import math
import json
import time
import tf

from std_msgs.msg import UInt32
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest

from Actions import Actions

class Task():
    """
    Task description
    ----------------
    The robot has to take out the garbage to the collection zone.

    Task requirements
    -----------------
    - Color Filter
    - Objects
    - Text to Speech
    - Manipulation
    - Navegation 
    """
       
    """docstring for Task"""
    
    def __init__(self):  
        lang = rospy.get_param('~lang', 'en')
        actions = Actions(lang)

        listener = tf.TransformListener()

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'TakeOutTheGarbage')
        rospy.set_param('/story/storyline',[
            ])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        # task variables
        locals = ['bin1_table','bin2','collection_zone']
        #bin1

        # speech variables
        self.srv = StartRequest()

        #actions.talk('Starting Take out the Garbage task')
        ########################################################################

        #Wait door opening to start
        #door = False
        #while not json.loads(actions.question('is_front_free').result):
        #    if not door:
        #        actions.talk('Knock knock')
        #        door = True
        #    continue
        #actions.talk('The door is open!')
        #actions.move('foward', seconds=5.0)

        actions.head("",2.9)
        for i in range(2):
            if i == 0:
                actions.goto('intermediario')
            actions.goto(locals[i])
            rospy.sleep(2)
            if i == 1:
                actions.head("",2.9)
            aux = False
            #actions.talk("Looking for a trash!")
            if i==0:
                coordinates = 0
                # while not aux:
                #     coordinates = actions.find_obj('closest')
                #     if coordinates is not None:
                #         actions.save_obj_location("lixo_local", coordinates, 0.7)
                #         #actions.talk("Saved!")
                #         aux = True
                #     if aux == True:
                #         print("Salvou local do objeto")
                #     else :
                #         print('No object detected')
            #actions.talk('Going to the bin')
            #time.sleep(2)
                #actions.goto('lixo_local')
            if i==1:
                rospy.sleep(2)
                actions.move('spin_right',0.45,4.06)
                actions.move('stop',0.0,0.0)
            rospy.sleep(2)
            actions.recog_garbage(5,0.2)
            actions.head('reset')
            actions.manip('find_trash')
            #rospy.sleep(3)
            #if resp == True:
            #actions.center_filter(0.1, 0.1)
            #actions.talk('Im at the right point')
            actions.manip('pick_lixo')
            if i == 0:
                actions.manip('guardar_lixo')

        time.sleep(2)
        #actions.talk('Going to the collection zone')
        actions.goto('collection_zone')
        actions.manip("find_trash")
        #actions.talk('Dropping the first trash')
        actions.manip("open")
        actions.manip("tirar_lixo")
        rospy.sleep(1)
        actions.manip("attack")
        rospy.sleep(1)
        #actions.talk('Dropping the second trash')
        actions.manip("open")
        actions.manip("home")

        # # actions.talk('Going to collection zone')
        # # actions.goto('collection_zone')
        # # actions.manip("", x=0.3, y=0.0, z=0.28, rx=0.0, ry=0.0, rz=0.0)
        # # time.sleep(1)
        # # actions.talk('Dropping the first trash')
        # # actions.manip("open")
        # # actions.dynamixel_goal(6, 2048)  
        # # actions.manip("home")

        """
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

        #######
        # talk = False
        # while not json.loads(actions.question('is_front_free').result):
        #     if not talk:
        #         actions.talk('Waiting to open the door')
        #         talk = True
        #     continue
        # actions.talk('The door is open!')
        # actions.move('foward', seconds=5.0)
        """
        #actions.talk('Task completed')

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
