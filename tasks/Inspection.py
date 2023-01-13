#!/usr/bin/env python3

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
        rospy.set_param('/story/title', 'Inspection')
        rospy.set_param('/story/storyline', [
            'Starting inspection!',
            'Going to inspection location',
            'Leaving arena'
            ])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        actions = Actions(lang)

        # task variables
        robotnames = ['Robot','Hera','Ivy']
        locals = json.loads(actions.question('know_places').result)
        specs = [
            '<robotnames>',
            'go to <locals>',
            'Start task',
            'Leave arena'
        ]

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = specs
        self.srv.choices.append(Opcs(id="robotnames", values=robotnames))
        self.srv.choices.append(Opcs(id="locals", values=locals))

        #
        self.inspection_location = 'inspection'
        self.end_location = 'end'

        ########################################################################
        ########################################################################

        # actions.pose('origin')
        # actions.goto('door')
        actions.talk('Hello! My name is HERA.')
        actions.talk('Please, talk to me after the signal.')

        while (True):
            try:
                speech = actions.hear(self.srv)
                #speech.result = 'Start task'                
                if (speech.result == '<robotnames>'):
                    person = actions.face()
                    actions.talk('Hello ' + person)
    
                elif(speech.result == 'go to <locals>'):
                    local = speech.choices[0].values[0]
                    actions.talk("Going to point1!")
                    actions.goto(point1)
                    actions.talk('I reach the location!')

                elif (speech.result == 'Start task'):
                    self.pub_vizbox_step.publish(0) # Starting inspection!
                    actions.talk('Starting inspection!')
                    talk = False
                    while not json.loads(actions.question('is_front_free').result):
                        if not talk:
                            actions.talk('Waiting to open the door')
                            talk = True
                        continue
                    actions.talk('The door is open!')
                    
                    self.pub_vizbox_step.publish(1) # Going to inspection location
                    actions.talk('Going to '+ self.inspection_location +' location!')
                    actions.move('foward', seconds=7.0)
                    actions.goto(self.inspection_location)
                    actions.talk('I reach the inspection location!')
                    actions.talk("I'm ready to leave the arena")
                    actions.talk("Just say Leave arena!")
                    rospy.sleep(5)
                    speech.result == 'Leave arena'
                elif (speech.result == 'Leave arena'):
                    self.pub_vizbox_step.publish(2) # Leaving arena
                    actions.talk('Leaving arena!')
                    actions.talk('Going to end!')
                    actions.goto(self.end_location)
                    actions.talk('Inspection finished!')

            except Exception as e:
                traceback.print_exc()

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
