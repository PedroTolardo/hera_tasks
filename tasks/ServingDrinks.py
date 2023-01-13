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

       
        actions.talk('Starting Serving Drinks task')
        actions.talk('Going to the living room')
        actions.goto('party')
        # actions.find_obj('closest')# encontrar pessoas com objetos
        # reconhecer 3 rostos das pessoas sem bebida como pessoa1, pessoa2 e pessoa3
        for i in range(1,4,1):
            var = False
            actions.goto('pessoa{}'.format(i)) #1a pessoa sem bebida
            actions.talk('Hello!')
            name = ''
            while name == '':
                actions.talk('Please, talk to me after the signal. What is your name?')
                speech = actions.hear(srv_names)
                name = speech.result
            drink_1 = ''
            while drink_1 == '':
                actions.talk('What is your favorite drink?')
                speech = actions.hear(srv_drinks)
                drink_1 = speech.result
            drink_2 = ''
            while drink_2 == '':
                actions.talk('What is your second favorite drink?')
                speech = actions.hear(srv_drinks)
                drink_2 = speech.result
            actions.talk('Excuse me.')
            actions.goto('bar')
            answer1 = ''
            while answer1 == '':
                actions.talk('Please, do you have' + drink_1)
                speech = actions.hear(srv_answer)
                answer1 = speech.result
            if answer1 == 'yes': 
                actions.talk('Please get me a' + drink_1) #tirar se precisar de mais tempo
                # actions.find_obj('closest')# encontrar bebida na mesa
                # if actions.find_obj('closest') != drink_1:
                #    actions.talk('Sorry, this is not what i asked for. Please get me a' + drink_1)  
            elif answer1 == 'no':
                var = True
                actions.talk('Then, please get me a' + drink_2)
                # actions.find_obj('closest')# encontrar bebida na mesa
                # if actions.find_obj('closest') != drink_1:
                    # actions.talk('Sorry, this is not what I asked for. Please get me a' + drink_2) 
            # actions.manip("closest")
            actions.manip("", x=0.3, y=0.0, z=0.28, rx=0.0, ry=0.0, rz=0.0)
            rospy.sleep(1)
            actions.manip("close")
            actions.manip("reset")
            actions.goto('party')
            actions.goto('pessoa{}'.format(i))
            actions.talk(name +'Here is your drink')
            if var == True:
                actions.talk('Sorry. They didnt have' + drink_1)
                actions.talk('so I brought you' + drink_2)
            actions.manip("", x=0.3, y=0.0, z=0.28, rx=0.0, ry=0.0, rz=0.0)
            actions.manip("open")       
        actions.talk('End task')

                
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
