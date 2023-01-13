#!/usr/bin/env python3

#from tarfile import _Bz2ReadableFileobj
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

class Receptionist:
    """
    Task description
    ----------------
    The robot has to take new guests to the living room to introduce them and offer a free place to sit.

    Task requirements
    -----------------
    - Speech Recognition
    - Text to Speech
    - Face Recognition
    - Manipulation
    - Navigation
    """
    def __init__(self):
        lang = rospy.get_param('~lang', 'en')
        actions = Actions(lang)

        listener = tf.TransformListener()

        # specs = [
        #     '<robotnames>',
        #     'Start task'
        # ]
        # speech variables
        self.srv =  StartRequest()
        #self.srv.spec = specs
        
        #nomes
        host_name = "James"
        host_drink = "Ice tea"
        name = ['','','']
        name_completo = ['', '','']
        srv_names = StartRequest()
        srv_names.spec  = ['James', 'Robert', 'John', 'Joan', 'Michael', 'David', 'William', 'Richard', 'Joseph', 'Thomas', 'Charles', 'Mary', 'Patricia', 'Jennifer', 'Linda', 'Elizabeth', 'Barbara', 'Susan', 'Jessica', 'Sarah', 'Karen']
        #drinks
        srv_drinks = StartRequest()
        srv_drinks.spec=  ['coconut water', 'Coke', 'Guarana','ice tea', 'tonic', 'water']
        drink = ['','','']
        drink_completo = ['', '', '']
        srv_resp = StartRequest()
        srv_resp.spec = ['yes', 'no']
        # Task
        actions.talk('Starting Receptionist Task')

        for i in range(3):
            # ============ LOCAL: RECEPTION ============
            actions.talk('Going to reception')
            #actions.goto('reception')
            actions.head("reset")
            actions.manip("home")
            actions.talk('Waiting for a new guest')
            while True:            
                bool = actions.face_check()
                if bool == "True":
                    break
            actions.talk('Hello! My name is HERA.')
            # saving name and face
            name[i] = ''
            name_completo[i] = ''
            actions.talk('Please, talk to me after the signal. What is your name?')
            speech = actions.hear(srv_names)
            name[i] = speech.result
            print(name[i])
            name[0] = 'John'
            name[1] = 'Michael'
            name[2] ='William'
            actions.talk(name[i])
            #if name[i]== '':
            #    name[i] = 'Michael'
            actions.talk('Please, take a step back and look at me.')
            actions.save_face(name[i])
            # saving drink
            drink[i] = ''
            drink_completo[i] = ''
            actions.talk('What is your favorite drink?')
            speech = actions.hear(srv_drinks)
            drink[i] = speech.result
            print(drink[i])
            drink[0] = 'Water'
            drink[1] = 'Coconut water'
            drink[2] = 'Ice tea'
            actions.talk('Í heard:'+ drink[i])
        
            # ============ LOCAL: SEATS ============    
            actions.talk('Ok! Just a second.')
            actions.goto('seats')
            actions.head("",3.3)
            actions.talk('Please, come close and stand on my left.')
            actions.talk('Everyone, please look at me.')
            print(name[i])
            #saving host's face and looking for spare seat
            if i == 0:
                #actions.save_face(host_name)
                recog,center_host,num = actions.face_recog(host_name)
                while True:
                    for i in range(5):
                        recog,center_host,num = actions.face_recog(host_name)
                        if center_host != 0.0:
                            break
                    if center_host == 0.0:
                            actions.move('foward', seconds=2.0)
                    else:
                        break
    
                seat_vazio = actions.lugar_vazio(center_host,0,0)
                #seat_vazio = 64
                #presenting host to guest & guest to host
                actions.talk('Hi' + host_name)
                actions.manip("point", x=1.5)
                actions.talk('This is ' + name[i] + '. Their favorite drink is ' + drink[i])
                actions.manip("point_people", center_host)
                actions.dynamixel('', 2, 'Goal_Position', 2000)
                actions.dynamixel('', 6, 'Goal_Position', 2800)
                actions.talk(name[i] + ', this is' + host_name + '. Their favorite drink is ' + host_drink)
            #looking for host, guest1 and spare seat
            if (i == 1):
                recog,center_host,num = actions.face_recog(host_name)
                recog,center_guest,num = actions.face_recog(name[0])
                seat_vazio = actions.lugar_vazio(center_host, center_guest, 0)
                #seat_vazio = 320
                #presenting host to guest & guest to host
                actions.talk('Hi everybody')
                actions.manip("point", x=1.5)
                actions.talk('This is ' + name[i] + '. Their favorite drink is ' + drink[i])
                actions.manip("point_people", center_host)
                actions.dynamixel('', 2, 'Goal_Position', 2000)
                actions.dynamixel('', 6, 'Goal_Position', 2800)
                actions.talk(name[i] + ', this is' + host_name + '. Their favorite drink is ' + host_drink)
                actions.manip("point_people", center_guest)
                actions.dynamixel('', 2, 'Goal_Position', 2000)
                actions.dynamixel('', 6, 'Goal_Position', 2800)
                actions.talk(name[i]+ 'This is ' + name[0] + '. Their favorite drink is ' + drink[0])
            if (i == 2):
                recog,center_host,num = actions.face_recog(host_name)
                recog,center_guest,num = actions.face_recog(name[0])
                recog,center_guest2,num = actions.face_recog(name[1])
                seat_vazio = actions.lugar_vazio(center_host, center_guest, center_guest2)
                #seat_vazio = None
                #presenting host to guest & guest to host
                actions.talk('Hi everybody')
                actions.manip("point", x=1.5)
                actions.talk('This is ' + name[i] + '. Their favorite drink is ' + drink[i])
                actions.manip("point_people", center_host)
                actions.dynamixel('', 2, 'Goal_Position', 2000)
                actions.dynamixel('', 6, 'Goal_Position', 2800)
                actions.talk(name[i] + ', this is' + host_name + '. Their favorite drink is ' + host_drink)
                actions.manip("point_people", center_guest)
                actions.talk(name[i]+ 'This is ' + name[0] + '. Their favorite drink is ' + drink[0])
                actions.manip("point_people", center_guest2)
                actions.talk(name[i]+ 'This is ' + name[1] + '. Their favorite drink is ' + drink[1])
            #pointing to spear seat
            if seat_vazio is None:
                actions.talk('Sorry, you will have to stand, there is no empty seat in the room')
            else:
                
                
                actions.manip("point_people",seat_vazio)
                actions.dynamixel('', 2, 'Goal_Position', 2000)
                actions.dynamixel('', 6, 'Goal_Position', 2800)
                actions.talk(name[i] + ', feel free to take a seat there')
                if seat_vazio == 64:
                    actions.talk('at the chair.')
                elif seat_vazio == 320:
                    actions.talk('at the left side of the sofa.')
                elif seat_vazio == 448:
                    actions.talk('at the right side of the sofa.')
                actions.manip("point", x=1)
    
        # End Task
        actions.manip("home")
        actions.head("reset")
        actions.talk("Ending Receptionist Task")   
        
                  

if __name__ == '__main__':
    try:
        rospy.init_node('receptionist')
        Receptionist()

    except KeyboardInterrupt:
        pass
