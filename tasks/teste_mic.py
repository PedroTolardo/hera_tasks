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
    - Navegation
    """
    def __init__(self):
        lang = rospy.get_param('~lang', 'en')
        actions = Actions(lang)

        listener = tf.TransformListener()

        #nomes
        host_name = "Charlie"
        host_drink = "Water"
        name = ['','']
        name_completo = ['', '']
        srv_names = StartRequest()
        names_listado = ['James', 'Robert', 'John', 'Michael', 'David', 'William', 'Richard', 'Joseph', 'Thomas', 'Charles', 'Mary', 'Patricia', 'Jennifer', 'Linda', 'Elizabeth', 'Barbara', 'Susan', 'Jessica', 'Sarah', 'Karen']
        srv_names.spec = ['']
        #drinks
        srv_drinks = StartRequest()
        srv_drinks.spec = ['']
        drinks_listado =  ['coconut water', 'coke', 'guarana','ice tea', 'tonic', 'water']
        drink = ['','']
        drink_completo = ['', '']
        # Task
        actions.talk('Starting Receptionist Task')

        for i in range(2):
            # saving name and face
            name[i] = ''
            name_completo[i] = ''
            actions.talk('Please, talk to me after the signal. What is your name?')
            speech = actions.hear(srv_names)
            name[i] = speech.result.split()
            print("Print pos split", name[i])
            for j in (name[i]):
                print("for 1", name_completo[i])
                for k in (names_listado):
                    print("achando robert", k)
                    if j == k:
                        print("condicao", k)
                        print("tentativa", i)
                        name_completo[i] = k
                        print("Teste", name_completo)
                        print("Teste 12", name_completo[i])
                        break
            if name_completo[i] == '':
                name_completo[i] = "Hunter"
            actions.talk('Please, take a step back and look at me.')
            # saving drink
            print("tentativa 222", i)
            print(name_completo[i])
            drink[i] = ''
            drink_completo[i] = ''
            actions.talk('What is your favorite drink?')
            speech = actions.hear(srv_drinks)
            drink[i] = speech.result.split()
            for j in (drink[i]):
                for k in (drinks_listado):
                    if j == k:
                        drink_completo[i] = k
            if drink_completo[i] == '':
                drink_completo[i] = "Water"
            actions.talk('This is ' + name_completo[i] + '. Their favorite drink is ' + drink_completo[i])
            actions.talk(name_completo[i] + ', this is' + host_name + '. Their favorite drink is ' + host_drink)
            #looking for host, guest1 and spare seat
            if (i == 1):
                actions.talk('Hi' + name_completo[0])
                actions.talk('This is ' + name_completo[i] + '. Their favorite drink is ' + drink_completo[i])
                actions.talk(name_completo[i]+ 'This is ' + name_completo[0] + '. Their favorite drink is ' + drink_completo[0])
            #pointing to spear seat
            actions.talk(name_completo[i] + ', feel free to take a seat there')
        
                  

if __name__ == '__main__':
    try:
        rospy.init_node('receptionist')
        Receptionist()

    except KeyboardInterrupt:
        pass