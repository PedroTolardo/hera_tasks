#!/usr/bin/env python3

from imp import acquire_lock
import traceback
from unicodedata import name
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
    """docstring for Task"""
    def __init__(self):
        rospy.init_node('Task')
        lang = rospy.get_param('~lang', 'en')
        actions = Actions(lang)
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        actions = Actions(lang)

        # task variables
        robotnames = ['Robot','Hera','Ivy']
        # locals = json.loads(actions.question('know_places').result)
        locals = ['party','reception','start']
        specs = [
            '<robotnames>',
            'Start task'
        ]  

        self.srv =  StartRequest()
        self.srv.spec = specs
        self.srv.choices.append(Opcs(id="robotnames", values=robotnames))
        self.srv.choices.append(Opcs(id="locals", values=locals))

        srv_names = StartRequest()
        srv_names.spec =  ['James', 'Robert', 'John', 'Michael','David', 'William', 'Richard', 'Joseph', 'Thomas', 'Charles', 'Mary','Patricia', 'Jennifer', 'Linda', 'Elizabeth', 'Barbara', 'Susan', 'Jessica', 'Sarah', 'Karen']
        # actions.goto('start')
        actions.manip('home')
        actions.head('reset')
        actions.talk('Starting People Recognition task')
        actions.talk('Waiting for a new guest')

        while True:
            face_check = actions.face_check()
            if face_check == 'True':
                break 

        actions.talk('Hello! My name is HERA, please, look at me and take a step back')
        actions.talk('Please, talk to me after the signal, what is your name?.')
        speech = actions.hear(srv_names)
        name_1 = speech.result
        while name_1 == '':
            actions.talk('Sorry, I did not understand that. Please, talk to me after the signal. What is your name?')
            speech = actions.hear(srv_names)
            name_1 = speech.result

        print(name_1)
        resp = actions.save_face(name_1)
        print(resp)
        
        actions.talk('Nice to meet you, ' + name_1 + '.')
        actions.move('spin_right',0.45,8.25)
        actions.move('stop',0.0,0.0)
        rospy.sleep(2)
        actions.talk('five seconds.')
        actions.manip('attack')

        rospy.sleep(5)
        actions.talk("Please, keep looking at me")
        center = 0.0
        actions.head('', 3.4)
        while center == 0.0:
            name, center, num = actions.face_recog(name_1)

        print("Center:" + str(center))
        success = actions.manip("point_people", center)
        if success == "success":
            actions.talk('Between '+ str(num) + ' there is ' + name + '.')


if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass

