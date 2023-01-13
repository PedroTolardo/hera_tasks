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
from sensor_msgs.msg import LaserScan

from Actions import Actions
#enter_bin=0


# def recog_garbage(msg):
#     rospy.loginfo(msg)
#     laser_ran=msg
#     amp_ang = 166  #Amplitude de leitura do laser(graus)
#     range_size = len(laser_ran) #Numero de medidas lidas pelo laser
#     lim_ang = 60 #Angulo para limitar a leitura
#     lim_laser = round(lim_ang*range_size/amp_ang) 
#     filter_range = laser_ran[int((range_size/2)-(lim_laser/2)):int((range_size/2)+(lim_laser/2))]#Leitura limitada
#     center_bin = filter_range.index(min(filter_range))
#     center_range = filter_range.index(filter_range[int(len(filter_range)/2)])
#     rospy.loginfo(center_bin)
#     rospy.loginfo(center_range)
    
#     if (center_range>center_bin+10):
#         rospy.loginfo("vai pra direita")
#         enter_bin = 2
#         actions.move('spin_right')#inverter

            
#     elif(center_range<center_bin-10):
#         rospy.loginfo("vai pra esquerda")
#         enter_bin=1
#         actions.move('spin_left')
#     else:
#         rospy.loginfo("chegou")
#         enter_bin=3
#         actions.move('stop')
#         pass




# def callback(msg):
#     if enter_bin!=3:
#         recog_garbage(msg.ranges)'
    
    
    


rospy.init_node('Task')
rospy.sleep(1)
lang = rospy.get_param('~lang', 'en')
# set vizbox story and publisher
rospy.set_param('/story/title', 'TakeOutTheGarbage')
rospy.set_param('/story/storyline',[])
pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)

actions = Actions(lang)
        
# task variables
robotnames = ['Robot','Hera','Ivy']
locals = ['bin1','bin2','collection_zone']
specs = [
        '<robotnames>',
        'Start task']

# speech variables
srv = StartRequest()
srv.spec = specs
srv.choices.append(Opcs(id="robotnames", values=robotnames))
srv.choices.append(Opcs(id="locals", values=locals))

actions =Actions(lang)
#rospy.Subscriber('/base_scan_front', LaserScan, callback)

actions.laser_line(0.3)
#recog_garbage
rospy.spin()
