#!/usr/bin/env python3

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
    """docstring for Task"""
    def __init__(self):
        rospy.init_node('Task')
        lang = rospy.get_param('~lang', 'en')
        
        listener = tf.TransformListener()

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'Receptionist')
        rospy.set_param('/story/storyline',[])
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

        # speech variables
        self.srv =  StartRequest()
        self.srv.spec = specs
        self.srv.choices.append(Opcs(id="robotnames", values=robotnames))
        self.srv.choices.append(Opcs(id="locals", values=locals))

        srv_names = StartRequest()
        srv_names.spec = ['Hunter', 'William', 'Amelia', 'Angel', 'Charlie', 'Jack', 'Ava', 'Charlotte', 'Max', 'Mia', 'Olivia', 'Parker', 'Sam', 'Noah', 'Oliver', 'Parker', 'Thomas']

        srv_drinks = StartRequest()
        srv_drinks.spec = ['Milk','Water', 'Coke','Tonic', 'Bubble Tea', 'Ice tea']
        # actions.goto('start')
        actions.talk('Starting Receptionist task')
        actions.goto('reception')
        actions.head("reset")
        actions.talk('Waiting for a new guest')
        rospy.sleep(2)
        actions.talk('Hello! My name is HERA.')
        name_1 = ''
        actions.talk('Please, talk to me after the signal. What is your name?')
        speech = actions.hear(srv_names)
        name_1 = speech.result
        if name_1 == '':
            name_1 = "Oliver"
        actions.talk('Please,look at me.')
        drink_1 = ''
        actions.talk('What is your favorite drink?')
        speech = actions.hear(srv_drinks)
        drink_1 = speech.result
        if drink_1 == '':
            drink_1 = "Coffee"
        actions.talk('Please, follow me')
        actions.goto('lugares')
        actions.head("",2.9)
        actions.talk('Please, stand on my right.')
        actions.manip("point", x=0.5)
        actions.talk('Hi Charlie, This is ' + name_1 + '. Their favorite drink is ' + drink_1)
        # actions.talk(name_1 + ' has black hair, is wearing a red t shirt and no face mask.' + name_1 + 'has aproximatelly 1 point six meters')
        actions.manip("point", x=1.1)
        actions.talk(name_1 + ', this is Charlie. Their favorite drink is ice tea')
        
        actions.manip("point", x=0.9)
        actions.dynamixel('', 2, 'Goal_Position', 2000)
        actions.dynamixel('', 6, 'Goal_Position', 2700)
        actions.talk(name_1 + ', feel free to take a seat there')
        actions.manip("point", x=1)
        actions.goto('reception')
        actions.head("reset")
        

        actions.talk('Waiting for a new guest')
        rospy.sleep(2)
        name_2 = ''
        actions.talk('Please, talk to me after the signal. What is your name?')
        speech = actions.hear(srv_names)
        name_2 = speech.result
        if name_2 == '':
            name_2 = "Max"
        actions.talk('Please,look at me.')
        drink_2 = ''
        actions.talk('What is your favorite drink?')
        speech = actions.hear(srv_drinks)
        drink_2 = speech.result
        if drink_2 == '':
            drink_2 = "Coke"
        actions.talk('Please, follow me')
        actions.head("",2.9)
        actions.goto('lugares')
        actions.talk('Please, stand on my right.')

        actions.talk('Hi Charlie and' + name_1)
        actions.manip("point", x=0.5)
        actions.talk('This is ' + name_2 + '. Their favorite drink is ' + drink_2)
        actions.manip("point", x=1.1)
        actions.talk(name_2 + ', this is Charlie. Their favorite drink is ice tea')
        
        actions.manip("point", x=0.9)
        actions.talk(name_2 + 'This is ' + name_1 + ' Their favorite drink is ' + drink_1)

        actions.manip("point", x=0.5)
        actions.talk(name_1 + 'This is ' + name_2 + ' Their favorite drink is ' + drink_2)
        actions.talk(name_2 + ' has black hair, is wearing a red t shirt and no face mask.' + name_2 + 'has aproximatelly 1 point six meters')
        
        actions.manip("point", x=0.6)
        actions.dynamixel('', 2, 'Goal_Position', 2000)
        actions.dynamixel('', 6, 'Goal_Position', 2700)
        actions.talk(name_2 + ', feel free to take a seat there')
        
        actions.manip("point", x=1)   
        actions.talk('End Task')

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass

