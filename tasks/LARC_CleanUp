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

class CleanUp:
    """
    Task description
    ----------------
    The robot has to clean up an Arena room with some misplaced objects, placing unknown objects in a garbage bin.
    
    Task requirements
    -----------------
    - Speech Recognition
    - Text to Speech
    - Object Recognition
    - Manipulation
    - Navegation
    """
    def __init__(self):
        lang = rospy.get_param('~lang', 'en')
        actions = Actions(lang)

        listener = tf.TransformListener()

        # speech variables
        self.srv =  StartRequest()
        
        srv_rooms = StartRequest()
        srv_rooms.spec = ['Bedroom', 'Office', 'Living Room', 'Kitchen']
        
        # Task
        actions.talk('Starting Clean Up Task')

        actions.talk('Waiting for a person')
        while True:            
            bool = actions.face_check()
            if bool == "True":
                break
        actions.talk('Hello! My name is HERA.')
        while room == '':
            actions.talk('Please, talk to me after the signal. What room would you like me to clean?')
            speech = actions.hear(srv_rooms)
            room = speech.result
            if room == '':
                actions.talk('Sorry. I could not understand that!')
    
        actions.talk('Ok, going to' + room)
        actions.head("",3.4)

        if room == "Bedroom":
            local1 = 'bedroom1'
            local2 = 'bedroom2'
            local3 = 'bedroom3'
        elif room == "Living room":
            local1 = 'living_room1'
            local2 = 'living_room2'
            local3 = 'living_room3'
        elif room == "Office":
            local1 = 'office1'
            local2 = 'office2'
            local3 = 'office3'
        elif room == "Kitchen":    
            local1 = 'kitchen1'
            local2 = 'kitchen2'
            local3 = 'kitchen3'

        #DETERGENTE
        manip_success = False
        actions.talk("Trying first location")
        #local1 - detergente
        actions.goto(local1)
        coordinates_detergente = actions.FindEspecificObject('detergente')
        pick_detergente(coordinates_detergente)
        if not manip_success:
            actions.talk("Trying second location")
            #local2 - detergente
            actions.goto(local2)
            coordinates_detergente = actions.FindEspecificObject('detergente')
            pick_detergente(coordinates_detergente)
            if not manip_success:
                actions.talk("Trying third location")
                #local3 - detergente
                actions.goto(local3)
                coordinates_detergente = actions.FindEspecificObject('detergente')
                pick_detergente(coordinates_detergente)
            else:
                actions.talk("Cleaner placed in its correct location")
        else:
            actions.talk("Cleaner placed in its correct location")


        #PANO
        manip_success = False
        actions.talk("Trying first location")
        #local1 - pano
        actions.goto(local1)
        coordinates_pano = actions.FindEspecificObject('pano')
        pick_pano(coordinates_pano)
        if not manip_success:
            actions.talk("Trying second location")
            #local2 - pano
            actions.goto(local2)
            coordinates_pano = actions.FindEspecificObject('pano')
            pick_pano(coordinates_pano)
            if not manip_success:
                actions.talk("Trying third location")
                #local3 - pano
                actions.goto(local3)
                coordinates_pano = actions.FindEspecificObject('pano')
                pick_pano(coordinates_pano)
            else:
                actions.talk("Cloth placed in its correct location")
        else:
            actions.talk("Cloth placed in its correct location")




def pick_detergente(self, coordinates_detergente):
    manip_success = False
    tentativa = 0

    if coordinates_detergente is not None:
            self.self.action.talk("I found ")
            while not manip_success:
                success_detergente = self.action.manip_goal(coordinates_detergente, "pick2")
                if success_detergente == "success":
                    self.action.talk('Pick succeded')
                    self.action.goto('mesa_kitchen')
                    self.action.laser_line(0.3)
                    success_place = self.action.manip('place3', 0.0)
                    i = 0
                    while not success_place == "success":
                        success_place = self.action.manip('place3', float(i))
                        i += 1
                    manip_success = True
                else:
                    self.action.talk('Pick failed')
                    self.action.manip('open')
                    self.action.manip('home')
                    self.action.talk('Trying again.')
                    if tentativa == 0:
                        self.action.move('left', seconds=1.0)
                    else:
                        self.action.move('right', seconds=2.0)
                        tentativa = 0


def pick_pano(self, coordinates_pano):
    manip_success = False
    tentativa = 0

    if coordinates_pano is not None:
            while not manip_success:
                success_pano = self.action.manip_goal(coordinates_pano, "pick2")
                if success_pano == "success":
                    self.action.talk('Pick succeded')
                    self.action.goto('mesa_kitchen')
                    self.action.laser_line(0.3)
                    success_place = self.action.manip('place3', 0.0)
                    i = 2
                    while not success_place == "success":
                        success_place = self.action.manip('place3', float(i))
                        i -= 1
                    manip_success = True
                else:
                    self.action.talk('Pick failed')
                    self.action.manip('open')
                    self.action.manip('home')
                    self.action.talk('Trying again.')
                    if tentativa == 0:
                        self.action.move('left', seconds=1.0)
                    else:
                        self.action.move('right', seconds=2.0)
                        tentativa = 0






if __name__ == '__main__':
    try:
        rospy.init_node('cleanup')
        CleanUp()

    except KeyboardInterrupt:
        pass