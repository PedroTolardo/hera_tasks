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
        rospy.sleep(1)
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
        locals = ['party','reception']
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
        actions.talk('Starting Receptionist task')
        actions.talk('Please, talk to me after the bip.')
        actions.pose('annoucement')
        actions.talk('Hello! My name is HERA.')
        actions.talk('Going to reception')
        
        actions.talk('Waiting for a new guest')
        
 
        aux = False
	while not aux:
            coordinates = actions.FindObject('closest')
            if not coordinates == None:
		aux = True
		actions.talk('Achei a coordenada')
		print coordinates
	    else:
		actions.talk('Nao achei a coordenada')
		
		                
            
        if coordinates is not None:
            actions.save_obj_location('lixo', coordinates)
            if True:
                actions.talk('Salvou o local')
            else:
                actions.talk('Nao salvou o local')



if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
