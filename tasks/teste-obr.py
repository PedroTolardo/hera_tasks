#!/usr/bin/env python3

from re import A
import traceback

from sklearn import manifold
import rospy
import math
import json
import time
import tf

from std_msgs.msg import UInt32, String
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest

from Actions import Actions

class Task():

    def callback_bot(self, msg):
        self.bot_ans = msg.data

    """docstring for Task"""
    def __init__(self):
        rospy.init_node('Task')
        lang = rospy.get_param('~lang', 'en')
        
        listener = tf.TransformListener()

        # set vizbox story and publisher
        rospy.set_param('/story/title', 'Receptionist')
        rospy.set_param('/story/storyline',[])
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

        self.bot_ans = None
        self.pub_bot = rospy.Publisher('/bot_input', String, queue_size=80)
        # self.pub_bot.publish(String(str(1)))
        self.sub_bot = rospy.Subscriber('/bot_output', String, self.callback_bot)
        
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

        srv_names = StartRequest()
        srv_names.spec = []

        #actions.head("",3.14)

        recog,center_host,num = actions.face_recog('mateus')
        print("center_host antes: ",center_host)
        recog,center_guest,num = actions.face_recog('pedro')
        #actions.goto('obr')
        print("center_host depois: ",center_host)
        print("center_guest : ",center_guest)
        seat_vazio = actions.lugar_vazio(center_host, center_guest, 0)
        print(seat_vazio)
        actions.manip('point_people', seat_vazio)
        # actions.goto('reception')
        # actions.talk('Hi everyone!')
        # actions.talk('Please, follow me!')
        # actions.goto('lugares')
        # actions.manip("point", x=1.3)
        # actions.talk('Please, make yourselfs confortable in this area.')
        # actions.dynamixel('', 2, 'Goal_Position', 2000)
        # actions.dynamixel('', 6, 'Goal_Position', 2700)
        # rospy.sleep(2)
        # actions.dynamixel('', 1, 'Goal_Position', 1800)
        # rospy.sleep(2)
        # actions.manip("home")
        # actions.manip("open")
        # # rospy.sleep(20)
        # actions.talk('Hello! My name is HERA. I am a domestic robot assistant, and I will show you what I am able to do.')
        # actions.talk('Starting Domestic Emergency task')
        # # actions.goto('start')
        # # rospy.sleep(3)
        # # actions.move('spin_left', 0.3 , 2.5)
        # # actions.move('stop')
        # rospy.sleep(3)
        # actions.head("",2.8)
        # rospy.sleep(1)
        # actions.talk('Oh no, I found a person on the floor!')
        # actions.goto('person')
        # actions.talk('I noticed that you are on the floor. Have you had an accident?')
        # speech = actions.hear(srv_names)
        # speech.result = 'yes'
        # actions.talk('Okay. Iniciating emergency procedure. Contacting the doctor.')
        # self.pub_bot.publish(String(str(1)))
        # rospy.sleep(5)
        # actions.talk('I need a few informations for triage. Please tell me what happened.')
        # speech = actions.hear(srv_names)
        # speech.result = 'I twisted my ankle'
        # self.pub_bot.publish(String(str(2)))
        # actions.talk('Okay. Dont worry! I will help you. Let me just take a picture so i can request a doctor.')
        # actions.manip("attack")
        # actions.dynamixel('', 6, 'Goal_Position', 1500)
        # rospy.sleep(2)
        # actions.save_face('quadrado')
        # actions.click()
        # self.pub_bot.publish(String(str(3)))
        # actions.talk('The picture was sent. Waiting for doctors recomendations.')
        # self.pub_bot.publish(String(str(4)))
        # self.bot_ans = 'yes'
        # rospy.sleep(10)
        # if(self.bot_ans == 'yes'):
        #     verifica = 'yes'
        #     self.bot_ans = None
        #     self.pub_bot.publish(String(str(5)))
        #     self.bot_ans = '60'
        #     rospy.sleep(10)
        #     num = self.bot_ans
        #     #actions.goto('pre_shelf')
        #     actions.talk('I gonna take the medicine')
        #     # actions.move('backward', 0.3, 2)
        #     actions.goto('shelf')
        #     resp = actions.manip('find_cima')
        #     rospy.sleep(3)
        #     if resp == True:
        #         actions.center_filter(0.1, 0.1)
        #         actions.manip('find_trash')
        #         rospy.sleep(1)
        #         actions.manip('med')
        #         rospy.sleep(1)
        #         actions.manip('attack')
        #     else:
        #         actions.manip('find_cima')
        #         actions.center_filter(0.1, 0.1)
        #         actions.manip('find_trash')
        #         rospy.sleep(1)
        #         actions.manip('med')
        #         rospy.sleep(1)
        #         actions.manip('attack')

        #     actions.move('backward', 0.3, 2)
        #     # actions.move('spin_left', 0.3 , 5)
        #     # actions.move('stop')
        #     actions.goto('person')
        #     rospy.sleep(0.5)
        #     actions.manip("place_remedio")
        #     actions.talk('Here is your medicine, please take it from my hand. Opening in 3 seconds...')
        #     rospy.sleep(3)
        #     actions.manip("open")
        #     rospy.sleep(1)
        #     actions.manip("home")
        #     actions.talk('Please, take' + str(num) + 'miligrams like the doctor requested. I am going to the door to wait for him.')
        # elif(self.bot_ans == 'no'):
        #     actions.talk('I am sorry. I am not autorized to give you medice. But dont worry! The doctor will be here shortly. I am going to the door to wait for him.')
           
        
        # actions.goto('reception')
        # actions.talk('Waiting for the doctor')
        # rospy.sleep(3)
        # actions.talk('Hello doctor. Thank you for coming! Please, follow me.')
        # actions.goto('person')
        # if verifica == 'yes':
        #     actions.talk('I am going to recall the doctor some of your personal information and let him know that you took the medicine.')
        # else:
        #     actions.talk('I am going to recall the doctor some of your personal information.')
        # actions.talk('I am going to say this is the local language so it is more clear for him.')

        # actions.move('spin_right', 0.3 , 2)
        # actions.move('stop')
        # actions.talk('Pacients name is Mike. He is 24 years old and weights 83 kilograms.', from_lang="en", to_lang="th")
        # actions.talk('Thank you for your help!')
        # actions.talk('End task')

if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass

