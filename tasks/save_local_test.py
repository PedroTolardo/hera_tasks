#!/usr/bin/env python3

import traceback
from turtle import delay

from cupshelpers import activateNewPrinter
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

        # actions.head("",2.7)
        # aux = False
        # speech variables
        # actions.move('foward', 0.3 , 4)

        actions.talk("Tchau Tchau pessoal!",from_lang="pt", to_lang="pt")
        actions.manip("open")
        actions.manip("close")
        actions.manip("open")
        actions.manip("close")
        actions.manip("open")
        actions.manip("close")
        # rospy.sleep(2)


        # actions.manip("open")
        # rospy.sleep(1)
        # actions.talk("Agora a casa está limpa",from_lang="pt", to_lang="pt")
        # actions.manip("home")
        # actions.talk("No que mais posso te ajudar?",from_lang="pt", to_lang="pt")
        # actions.talk("Vou levar o lixo para fora",from_lang="pt", to_lang="pt")
        # srv_names = StartRequest()
        # srv_names.spec = ['Hunter', 'William', 'Wagner', 'Caio']
        # name_1 = ''
        # while name_1 == '':
        #     actions.talk("Whats your name ?")
        #     speech = actions.hear(srv_names)
        #     name_1 = speech.result
            
        # actions.save_face(name_1)

        # actions.talk("Start face Recognition")
        # (recog,center) = actions.face_recog('flora')
        # actions.talk("Hello " + str(recog))
        # actions.manip("point_people",center)
        
        # (recog,center) = actions.face_recog('gui')
        # actions.talk("Hello " + str(recog))
        # actions.manip("point_people",center)

        # (recog,center) = actions.face_recog('nicolas')
        # actions.talk("Hello " + str(recog))
        # actions.manip("point_people",center)


        # actions.save_obj_location("lixo_1_0", coordinates)
        # aux = False
        # actions.talk("Starting Save local task!")
        # actions.talk("Looking for an object!")
        # coordinates = 0
        # while not aux:
        #     coordinates = actions.find_obj('closest')
        #     print("coordenDA ",coordinates)
        #     if coordinates is not None:
        #         actions.talk("saving ...")
        #         actions.save_obj_location("lixo_local", coordinates, 0.70)
        #         #actions.save_obj_location("lixo_local", coordinates)
        #         actions.talk("Saved!")
        #         aux = True
        #     if aux == True:
        #         print("Salvou local do objeto")
        #     else :
        #         print('No object detected')
        # rospy.loginfo(coordinates)
        # actions.talk('Im going to that position')
        # actions.goto('lixo_local')
        # actions.talk('Done!')
        # actions.talk("Starting Save local task!")
        # actions.head("", 3.14)
        # resp = actions.manip('find_trash')
        # rospy.sleep(3)
        # if resp == True:
        #     actions.center_filter(0.15, 0.1)
        #     actions.talk('Im at the right point')
        #     actions.manip('pick_trash')
        # else:
        #     actions.manip('find_trash')
        #     actions.center_filter(0.15, 0.1)
        #     actions.talk('Im at the right point')
        #     actions.manip('pick_trash')
        # srv_names = StartRequest()
        # srv_names.spec = ['Hunter', 'William', 'Wagner', 'Caio']
        # name_1 = ''
        # while True:            
        #     bool = actions.face_check()
        #     print("bool: ",bool)
        #     if bool == "True":
        #         break
        # if bool == True:
        #     actions.talk('Hello, whats your name?')
        #     while name_1 == '':
        #         speech = actions.hear(srv_names)
        #         name_1 = speech.result
        # coord = None
        # while coord == None:
        #     coord = actions.FindSpecificObject('cookie')
        #     print("coord: ",coord)


        # print(coord)

        # if coord != {0:[0,0,0]}:
        #     actions.talk('Got it ')
        
          


        # (recog,center) = actions.face_move_recog("guigui",3)
        # print("Recognized: ",recog)
        # print("Center: ",center)


    
	

        



        


if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
