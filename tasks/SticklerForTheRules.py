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

rule_forbi = "You are breaking the forbidden room rule. Please follow me to the next room to fix it."
rule_drink = "You are breaking the compulsory hidatrion rule. Please follow me to the kitchen so i can give you a drink."
rule_trash = "You are breaking the no literring rule. Please take the garbage and and follow me."
rule_shoes = "You are breaking the no shoes rule. Please follow me to the entrance door to take out your shoes. "


class Task():
       
    """docstring for Task"""
    def __init__(self):        
        rospy.sleep(1)
        lang = rospy.get_param('~lang', 'en')

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'SticklerForTheRules')
        rospy.set_param('/story/storyline',[
            ])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        self.actions = Actions(lang)

        # task variables
        specs = ['yes', 'no']

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = specs

        ########################################################################
        ########################################################################         

        self.actions.talk('Starting Stickler For The Rules task')

        ########################################################################         
        ########################################################################       
          
        # self.actions.talk('Rule number 1: Forbidden room')
        self.actions.talk('Checking forbidden room')
        self.actions.goto('forbidden_room')
        self.actions.talk('I reached forbidden room')
        self.actions.talk('Looking for guests')

        if(not self.found_person()):
            self.actions.talk('There is no guest breaking a rule here!.')
            self.actions.goto('forbidden_room2')
            if(not self.found_person()):
               self.actions.talk('There is no guest breaking a rule here!.')
            else:
                self.guide_person('party2', rule_forbi)
        else:
            self.guide_person('party2', rule_forbi)

        ############### Remover na primeira tentativa #######################
        if(not self.found_person()):
            self.actions.talk('Oh no. the guest didnt follow me.')
            self.actions.talk('Going back to forbidden room.')
            self.actions.goto('forbidden_room3')
            self.actions.talk('I told you to follow me.')
            self.actions.talk('Please, follow me to another room.')
            self.actions.goto('party2')
        ############### Remover na primeira tentativa #######################

        self.actions.talk('Thank you for respecting the rule.')

        ########################################################################         
        ########################################################################         

        # self.actions.talk('Rule number 2: Compulsory hydratation')

        self.actions.talk('since you are already here, let me ask you a question.')
        speech = ''
        self.actions.talk('Please, answer yes or no after the signal. Do you have a drink?')
        speech = self.actions.hear(self.srv)  
        
        if speech == '':          
            speech = 'no'

        if (speech.result == 'yes'):
            self.actions.talk('You said yes. please, enjoy the party')
            self.actions.talk('Ok, going to party room.')
            self.actions.goto('party1')
            if(self.found_person()):
                self.actions.talk('Please, answer yes or no after the signal. Do you have a drink?')
                speech = self.actions.hear(self.srv)  
                if speech == '':          
                    speech = 'no'
                if (speech.result == 'yes'):
                    self.actions.talk('You said yes.')
                    self.actions.talk('Ok. please, enjoy the party.')

        elif (speech.result == 'no'):
            self.actions.talk('You said no.')
            self.guide_person('bar', rule_drink)


            coordinates = self.actions.find_obj('closest')
            if coordinates is not None:
                self.actions.manip("point", x=0.6)
                self.actions.dynamixel('', 2, 'Goal_Position', 2000)
                self.actions.dynamixel('', 6, 'Goal_Position', 2700)
                self.actions.talk('Please, feel free to take your favorite drink and enjoy the party')
                self.actions.manip("home")
                self.actions.manip("open")
                rospy.sleep(2)

                speech = ''
                self.actions.talk('Please, answer yes or no after the signal. Did you get your drink?')
                speech = self.actions.hear(self.srv)  
                
                if speech == '':          
                    speech = 'yes'

                if (speech.result == 'yes'):
                    self.actions.talk('Ok. please, enjoy the party.')
                elif (speech.result == 'no'):
                    self.actions.talk('Please, take your drink')

                self.actions.talk('Going back to party room.')
                self.actions.goto('party1')
            else:
                self.actions.talk('Oh no. the guest didnt follow me.')
                self.actions.talk('Going back to party room.')
                self.actions.goto('party1')

        ########################################################################         
        ########################################################################         

        # self.actions.talk('Rule number 3: no littering')
        self.actions.head("",2.5)
        self.actions.talk('Looking for trash')

        coordinates = self.actions.find_obj('closest')
        if coordinates is not None:
            self.actions.talk("I found a trash")
            self.actions.save_obj_location("lixo", coordinates, 1.00)
            self.actions.talk('Im going to that position')
            self.actions.goto('lixo')
            self.actions.talk('Done!')
            self.actions.talk('Hello, guest')
            self.actions.talk(rule_trash)
            self.actions.goto('trash')
            self.actions.talk('Please, throw the garbage in this bin.')
            self.actions.manip("point", x=1.2)
            self.actions.dynamixel('', 2, 'Goal_Position', 2000)
            self.actions.dynamixel('', 6, 'Goal_Position', 2700)
            rospy.sleep(3)

            speech = ''
            self.actions.talk('Please answer yes or no after de signal. Did you throw the garbage? ')
            speech = self.actions.hear(self.srv)  

            if speech == '':          
                speech = 'yes'

            if (speech.result == 'yes'):
                self.actions.talk('You said yes.')
                self.actions.talk('Please, enjoy the party')
            elif (speech.result == 'no'):
                self.actions.talk('You said no.')
                self.actions.talk('Please throw the garbage in the bin.')
            self.actions.talk('Ok!')

        else:
            self.actions.goto('kitchen')

            self.actions.talk('Looking for trash')
            coordinates = self.actions.find_obj('closest')
            if coordinates is not None:
                self.actions.talk("I found a trash")
                self.actions.save_obj_location("lixo", coordinates, 1.00)
                self.actions.talk('Im going to that position')
                self.actions.goto('lixo')
                self.actions.talk('Done!')
                self.actions.talk('Hello, guest')
                self.actions.talk(rule_trash)
                self.actions.goto('trash')
                self.actions.talk('Please, throw the garbage in this bin.')
                self.actions.manip("point", x=1.2)
                self.actions.dynamixel('', 2, 'Goal_Position', 2000)
                self.actions.dynamixel('', 6, 'Goal_Position', 2700)
                rospy.sleep(3)

                speech = ''
                self.actions.talk('Please answer yes or no after de signal. Did you throw the garbage? ')
                speech = self.actions.hear(self.srv)  

                if speech == '':          
                    speech = 'yes'

                if (speech.result == 'yes'):
                    self.actions.talk('You said yes.')
                    self.actions.talk('Please, enjoy the party')
                elif (speech.result == 'no'):
                    self.actions.talk('You said no.')
                    self.actions.talk('Please throw the garbage in the bin.')
            
            self.actions.talk('Ok!')

        ########################################################################         
        ########################################################################         

        # self.actions.talk('Rule number 4: no shoes inside the house')

        self.actions.goto('party1')

        rospy.sleep(10)

        self.actions.talk("looking for shoes in the house.")

        if(not self.found_person()):
            self.actions.talk('There is no guest breaking a rule here!.')
        else:            
            self.actions.goto('person')
            rospy.sleep(5)
            self.actions.head("",2.5)
            rospy.sleep(3)
            self.actions.head("",3.14)
            self.actions.talk(rule_shoes)
            self.actions.goto('entrance')
            self.actions.talk("please, remove your shoes.")
            rospy.sleep(3)

            speech = ''
            self.actions.talk('Please answer yes or no after de signal. Did you take of your shoes?')
            speech = self.actions.hear(self.srv)  

            if speech == '':          
                speech = 'yes'

            if (speech.result == 'yes'):
                self.actions.talk('You said yes.')
                self.actions.talk('Please, enjoy the party')
            elif (speech.result == 'no'):
                self.actions.talk('You said no.')
                self.actions.talk('Please take of your shoes.')


        self.actions.talk('Task completed')
                

    def found_person(self):
        coordinates = None
        for i in range(10):
            coordinates = self.actions.find_obj('closest')

        if(coordinates is not None):
            self.actions.save_obj_location("person", coordinates, 1.00)
            return True
        else:
            return False

    def guide_person(self, local, rule):
        self.actions.talk("I found a guest breaking a rule")
        self.actions.talk('Im going to that position')
        self.actions.goto('person')
        self.actions.talk('Done!')
        self.actions.talk('Hello guest.')
        self.actions.talk(rule)
        self.actions.goto(local)


if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
