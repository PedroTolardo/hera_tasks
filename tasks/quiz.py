#!/usr/bin/env python3

import math
import time
import rospy
import json
import traceback
import sys
from random import randint
from gsr_ros.srv import StartRequest
from gsr_ros.msg import Opcs
from std_msgs.msg import String
from std_msgs.msg import UInt32

# from hera.robot import Robot as HERA
from Actions import Actions
from agent.util.enuns import Side
#from spr_data import *



class Task():
    """docstring for SPR"""
    def __init__(self):   
        # set vizbox story
        rospy.set_param('/story/title', 'Speech task')
        rospy.set_param('/story/storyline', ['Riddles  game'])
        lang = rospy.get_param('~lang', 'en')
        # vizbox publishers
        self.pub_vizbox_step = rospy.Publisher('/challenge_step', UInt32, queue_size=80)
        pub = rospy.Publisher('quizz', String, queue_size=10)
        actions = Actions(lang)
        
        # start agent
        # self.hera = HERA()
        # report.title = 'SPR Report'
        
        self.questions = ['What is the heros name in The Legend of Zelda';
                          'What are the names of the ghosts who chase Pac Man and Ms. Pac Man';
                          'Whats the name of the Mythbusters crash test dummy';
                          'What is an Oxford comma';
                          'Who was the captain of the Enterprise in the pilot episode of Star Trek';
                          'What is the symbol for the modulus operator in C';
                          'What function is automatically called at the beginning of a C++ program';
                          'Which computer programming languages was introduced by IBM in 1957';
                          'Who is considered as the first programmer';
                          'Has a robot ever killed a person';
                          'Who was HitchBOT';
                          'Are self-driving cars safe';
                          'Who invented the compiler';
                          'Who created the Python Programming Language';
                          'Is Mark Zuckerberg a robot';
                          'Why did you run away';
                          'What kind of salad do robots like';
                          'What did you ate for lunch';
                          'Why did robots get angry so often';
                          'Why should not R2D2 be allowed in movies';
                          'What is your favorite style of music';
                          'What does the acronym SMTP represent';
                          'What does the acronym MPEG represent';
                          'What does the acronym GiMP represent';
                          'What does the acronym GNU represent';
                          'What is the most populous city in Brazil';
                          'Which continent is Brazil located in';
                          'On  what  day,  month  and  year  was  Brazil independence  declared';
                          'How many states does Brazil have (with Federal District)';
                          'In what year did the first Brazilian astronaut go to space';
                          'What is the only capital of Brazil crossed by the Equator';
                          'How many time zones are there in Brazil';
                          'In which city is the world first urban elevator and what is the name of that elevator';
                          'What is the only biome present in Brazil that is exclusive in the world';
                          'Pampulha Lake is a tourist spot in which Brazilian city';
                          'What is the smallest Brazilian state in territorial extension';
                          'Which capitals in Brazil have the same name as your state';
                          'Where is the Itamaraty Palace located';
                        'What was the first name given to Brazil by the Portuguese';
'What is the Newest State in Brazil';
'What is the oldest state in Brazil';
'What is the capital of Ceara';
'What is the capital of Rio Grande do Sul';
'What is the capital of Rio Grande do Norte';
'What is the capital of Brazil';
'What is the capital of Pernambuco';
'What is the capital of Para';
'What is the origin of the name of the city of São Bernardo do Campo';
'Who were the first residents of São Bernardo do Campo';
'How old is the city of São Bernardo do Campo';
'What is the area of São Bernardo do Campo';
'How many neighborhoods does the city of São Bernardo do Campo have';
'Who was the first mayor of the city of São Bernardo do Campo';
'São Bernardo do Campo borders with which cities';
'What is the name of someone born in São Bernardo do Campo';
'Who founded the FEI';
'What does the roof of the Capela Santo Inácio de Loyola represent';
'How many buildings do we have at the FEI';
'How many partnerships does the FEI have';
'FEI is recognized as the largest educational hub in Latin America than';
'What was the first name of the FEI';
'What decade did the FEI come to SBC';
'What does the name FEI mean';
'What does AGFEI mean';
'What is the ROBOFEI project about';
'What is the FEIBAJA project about';
'What is the AERODESIGN project about';
'What is the CONCRETO-FEI project about';
'What is the FORMULA-FEI project about';
'What is the PROGRAMMING MARATHON project about';
'What is the AICHE-FEI project about';
'What is the VR-FEI project about';
                          ]


        self.answer = ['Despite most peoples believes, he is called Link',
                        'Inky, Blinky, Pinky, and Clyde',
                        'The Mythbusters crash test dummy is called Buster',
                        'The hotly contested punctuation before a conjunction in a list',
                        'The captain of the Enterprise in the pilot episode was Captain Pike',
                        'The percentage symbol is used as modulus operator in C',
                        'The main function',
                        'Fortran was introduced by IBM in 1957',
                        'The first programmer was Ada Lovelace',
                        'The first known case of robot homicide occurred in 1981, when a roboticarm crushed a Japanese Kawasaki factory worker',
                        'A hitchhiking robot that relied on the kindness of strangers to travel theworld and was slain by humans',
                        'Yes.  Car accidents are product of human misconduct',
                        'Grace Hoper.  She wrote it in her spare time',
                        'Python was invented by Guido van Rossum',
                        'Sure.  I have never seen him drink water',
                        'I heard an electric can opener',
                        'Salads made with ice-borg lettuce',
                        'I had a byte',
                        'People kept pushing our buttons',
                        'He says so many foul words they have to bleep everything he says!',
                        'I like electronic and heavy Metal',
                        'SMTP stands for Simple Mail Transport Protocol',
                        'MPEG stands for Moving Picture Experts Group',
                        'GNU Image Manipulation Program',
                        'GNU is a recursive acronym meaning GNU is Not Unix',
                        'Sao Paulo is the most populous city in Brazil with 12.03 million residents',
                        'The Brazilian territory is located on the South American continent',
                        'On September 7, 1822, Brazils independence was declared.',
                        'Currently, Brazil is divided into 26 states and the Federal District, alto-gether there are 27 federative units',
                        'In March 2006, Pontes became the first Brazilian to go to space.',
                        'Macapa is the only Brazilian capital crossed by the Equator line.',
                        'Brazil  is  a  country  with  continental  dimensions,  in  all  it  has  four  timezones',
                        'The  Lacerda  Elevator  is  a  public  urban  elevator  located  in  Salvador, Brazil.',
                        'The Caatinga, characterized by its dry, desert habitat is the only one of Brazils biomes found exclusively within the country',
                        'Belo Horizonte',
                        'Sergipe',
                        'Sao Paulo and Rio de Janeiro',
                        'Brasilia',
                        'Ilha de vera Cruz',
                        'Bahia',
                        'Amazonas',
                        'Rio de Janeiro',
                        'Tocantins',
                        'Pernambuco',
                        'Fortaleza',
                        'Porto Alegre',
                        'Natal',
                        'Brasilia',
                        'Recife',
                        'Belem',
]

        self.robotnames = ['Robot', 'Hera', 'Ivy']
        #self.locals = json.loads(actions.question('know_places').result)

        self.spec = [
            '<robotnames>',
            'Start task']

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = self.spec
        self.srv.choices.append(Opcs(id="robotnames", values=self.robotnames))
        #self.srv.choices.append(Opcs(id="locals", values=self.locals))

        actions.talk('Start Task!')
        talk = False
        #while not json.loads(actions.question('is_front_free').result):
        #    if not talk:
        #        actions.talk('Waiting to open the door')
        #        talk = True
        #    continue
        #actions.talk('The door is open!')
        #actions.move('foward', seconds=5.0)
        #actions.goto('question')

    
        try:
            speech = actions.hear(self.srv)

        
            actions.talk('Starting speech recognition task!')
            # #actions.talk('Who want to play riddles with me?')
            time.sleep(5)
            # actions.talk('Please, ask me something only after the beep or wait for the next beep.')

            count = 1
            while count <= 6:
                pub.publish(str(count))
                try:
                    actions.talk('Please, make your question after the signal.')
                    srv_question = StartRequest()
                    srv_question.spec = self.questions
                    speech = actions.hear(srv_question)
                    if speech.result != "":
                        if speech.result in self.questions:
                        # confirm = ''
                        # while not confirm == 'yes':
                        #     actions.talk('Did you ask'+speech.result)
                        #     actions.talk('Say yes or no')
                        #     srv_confirm = StartRequest()
                        #     srv_confirm.spec = ['yes','no']
                        #     speech = actions.hear(srv_confirm)
                        #     follow = speech.result.lower()
                        # if confirm == 'yes':
                        #     index = self.questions.index(speech.result)
                        #     answer = self.answer[index]
                        # else:
                        #     continue
                            index = self.questions.index(speech.result)
                            answer = self.answer[index]
                    	else:
                        	answer = "Sorry, I could not find an answer to that"
                    actions.talk(answer)
                    count += 1
                    


                except Exception as e:
                    rospy.logerr(e.message)
                    traceback.print_exc()
            time.sleep(5)
            actions.talk('Finish riddles game')
        except KeyboardInterrupt:
            rospy.loginfo('Shuting down..')
            traceback.print_exc()
            

if __name__ == '__main__':
    global variable
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
