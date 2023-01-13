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
        locals = ['bin1','bin2','collection_zone']
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

        #actions.head("",2.9)
        #for i in range(2):
            #if i == 0:
            #    actions.goto('intermediario')
        #actions.goto(locals[1])
        ##actions.talk("Looking for a trash!")
        #actions.head("",2.9)
        #rospy.sleep(2)
        ##actions.talk('Going to the bin')
        #actions.recog_garbage(5,0.2)
        #actions.head('reset')
        #actions.manip("tira_tampa", x = float(1))
        #rospy.sleep(2)
        #actions.manip('find_trash')
        #actions.manip('pick_lixo')
        ##if i == 0:
        ##    actions.manip('guardar_lixo')
#
        #time.sleep(2)
        ##actions.talk('Going to the collection zone')
        #actions.goto('collection_zone')
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

        #actions.talk('Task completed')

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
