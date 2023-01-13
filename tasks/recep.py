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

        actions.goto('start')
        actions.talk('Starting Receptionist task')
        actions.goto('reception')
        actions.head("reset")
        actions.talk('Waiting for a new guest')
        while True:            
            bool = actions.face_check()
            if bool == "True":
                break
        
        actions.talk('Hello! My name is HERA.')
        name_1 = ''
        actions.talk('Please, talk to me after the signal. What is your name?')
        speech = actions.hear(srv_names)
        name_1 = speech.result
        if name_1 == '':
            name_1 = "Hunter"
        actions.talk('Please,look at me.')
        actions.save_face(name_1)
        drink_1 = ''
        actions.talk('What is your favorite drink?')
        speech = actions.hear(srv_drinks)
        drink_1 = speech.result
        if drink_1 == '':
            drink_1 = "Water"
        actions.talk('Please, follow me')
        actions.goto('lugares')
        actions.head("",2.7)
        recog,center_host = actions.face_recog("")
        pose = 0
        if recog is not None:
            actions.save_face("Charlie")
            pose = 1
        else:
            actions.goto('lugares2')
            recog,center_host = actions.face_recog("")
            if recog is not None:
                actions.save_face("Charlie")
                pose = 2
            else:
                actions.goto('lugares3')
                recog,center_host = actions.face_recog("")
                if recog is not None:
                    actions.save_face("Charlie")
                    pose = 3
        
        if pose == 1 or pose == 2:
            actions.talk('Please, stand on my right.')
            actions.talk('Hi Charlie.')
            actions.manip("point", x=0.5)
        else:
            actions.talk('Please, stand on my left.')
            actions.talk('Hi Charlie.')
            actions.manip("point", x=1.5)
        
        actions.talk('This is ' + name_1 + '. Their favorite drink is ' + drink_1)
        recog,center_host = actions.face_recog("Charlie")
        actions.manip("point_people",center_host)
        actions.talk(name_1 + ', this is Charlie. Their favorite drink is ice tea')
        seat_vazio = actions.lugar_vazio(center_host,0)
        actions.manip("point_people",seat_vazio)
        actions.dynamixel('', 2, 'Goal_Position', 2000)
        actions.dynamixel('', 6, 'Goal_Position', 2700)
        actions.talk(name_1 + ', feel free to take a seat there')
        actions.manip("point", x=1)
        actions.goto('reception')
        actions.head("reset")

        actions.talk('Waiting for a new guest')
        while True:            
            bool = actions.face_check()
            if bool == "True":
                break
      
        name_2 = ''
        actions.talk('Please, talk to me after the signal. What is your name?')
        speech = actions.hear(srv_names)
        name_2 = speech.result
        if name_2 == '':
            name_2 = "Hunter"
        actions.talk('Please,look at me.')
        drink_2 = ''
        actions.talk('What is your favorite drink?')
        speech = actions.hear(srv_drinks)
        drink_2 = speech.result
        if drink_2 == '':
            drink_2 = "Water"
        actions.talk('Please, follow me')
        actions.head("",2.7)
        actions.goto('lugares')
        actions.talk('Please, stand on my right.')
        recog,center_host = actions.face_recog("Charlie")
        letra = ""
        if recog is not None:
            host_pose = 1
        else:
            actions.move('spin_right', vel = 0.8, seconds=2.0)
            actions.move('stop',0,0)
            recog,center_host = actions.face_recog("Charlie")
            if recog is not None:
                host_pose = 2
            else:
                actions.move('spin_left', vel = 0.8, seconds=4.0)
                actions.move('stop',0,0)                
                recog,center_host = actions.face_recog("Charlie")
                if recog is not None:
                    host_pose = 3
        
        recog,center_host = actions.face_recog(name_1)
        letra = ""
        if recog is not None:
            guest_pose = 1
        else:
            actions.goto('lugares2')
            recog,center_host = actions.face_recog(name_1)
            if recog is not None:
                guest_pose = 2
            else:
                actions.goto('lugares3')
                recog,center_host = actions.face_recog(name_1)
                if recog is not None:
                    guest_pose = 3

        if host_pose == 1 and guest_pose == 1:
            actions.goto('lugares3')
            actions.manip('point',x=1.5)

        elif host_pose == 2 and guest_pose == 2 or host_pose == 3 and guest_pose == 3:
            actions.goto('lugares')
            actions.manip('point',x=1.5)

        elif host_pose == 1 and guest_pose == 2:
            actions.goto('lugares')
            recog,center_host = actions.face_recog("Charlie")
            seat_vazio = actions.lugar_vazio(center_host,0)
            actions.manip("point_people", center_host)
        
        actions.goto('lugares3')
        if letra == "cg":
            actions.manip("point_people", 480)
            actions.dynamixel('', 2, 'Goal_Position', 2000)
            actions.dynamixel('', 6, 'Goal_Position', 2700)
            actions.talk(name_2 + ', feel free to take a seat there')
            actions.talk("End task")
            pass
        elif letra == "c":
            recog,center_guest = actions.face_recog(name_1)
            if recog is not None:
                actions.talk('Hi' + name_1 +'. This is ' + name_2 + '. Their favorite drink is ' + drink_2)
                letra = "cg"
            else:
                actions.goto('lugares2')
        elif letra == "g":
            recog,center_host = actions.face_recog("Charlie")
            if recog is not None:
                actions.talk('Hi Charlie. This is ' + name_2 + '. Their favorite drink is ' + drink_2)
                letra = "cg"
            else:
                actions.goto('lugares2')
        
        if letra == "cg":
            actions.manip("point_people", 160)
            actions.dynamixel('', 2, 'Goal_Position', 2000)
            actions.dynamixel('', 6, 'Goal_Position', 2700)
            actions.talk(name_2 + ', feel free to take a seat there')
        elif letra == "c":
            recog,center_guest = actions.face_recog(name_1)
            if recog is not None:
                actions.talk('Hi' + name_1 +'. This is ' + name_2 + '. Their favorite drink is ' + drink_2)
                seat_vazio = actions.lugar_vazio(center_guest,0)
        elif letra == "g":
            recog,center_host = actions.face_recog("Charlie")
            if recog is not None:
                actions.talk('Hi Charlie. This is ' + name_2 + '. Their favorite drink is ' + drink_2)
                seat_vazio = actions.lugar_vazio(center_host,0)




            actions.goto('lugares2')
            recog,center_host = actions.face_recog("")
            if recog is not None:
                actions.save_face("Charlie")
                pose = 2
            else:
                actions.goto('lugares3')
                recog,center_host = actions.face_recog("")
                if recog is not None:
                    actions.save_face("Charlie")
                    pose = 3



        actions.talk('Hi everyone!')
        actions.manip("point", x=1.5)
        actions.talk('This is ' + name_2 + '. Their favorite drink is ' + drink_2)
        recog,center_host = actions.face_recog("Charlie")
        actions.manip("point_people",center_host)
        actions.talk(name_2 + ', this is Charlie. Their favorite drink is ice tea')
        recog,center_guest = actions.face_recog(name_1)
        actions.manip("point_people",center_guest)
        actions.talk(name_2 + 'This is ' + name_1 + ' Their favorite drink is ' + drink_1)

        seat_vazio2 = actions.lugar_vazio(center_host,center_guest)
        actions.manip("point_people",seat_vazio2)
        actions.dynamixel('', 2, 'Goal_Position', 2000)
        actions.dynamixel('', 6, 'Goal_Position', 2700)
        actions.talk(name_2 + ', feel free to take a seat there')
        
        actions.manip("point", x=1)   
        actions.talk('End')

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
