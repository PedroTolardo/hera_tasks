#!/usr/bin/env python3
from Actions import Actions
import json
import rospy
import traceback
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest


class SpeechRecognition:
    """
    Task description
    ----------------
    The robot must answer a set of questions to an operator at the first attempt without asking for confirmation
    Task requirements
    -----------------
    - Speech Recognition
    - Text to Speech
    """

    def __init__(self):
        lang = rospy.get_param('~lang', 'en')
        actions = Actions(lang)

        # Get questions and answers from file, separated by ;
        with open('/home/robofei/catkin_hera/src/hera_robot/hera_tasks/tasks/resources/speech_recognition/questions.txt', 'r') as f:
            self.questions = f.read().split(';')
            self.questions = [q.strip() for q in self.questions]
            self.questions = list(filter(None, self.questions))

        with open('/home/robofei/catkin_hera/src/hera_robot/hera_tasks/tasks/resources/speech_recognition/answers.txt', 'r') as f:
            self.answers = f.read().split(';')
            self.answers = [a.strip() for a in self.answers]
            self.answers = list(filter(None, self.answers))

        with open('/home/robofei/catkin_hera/src/hera_robot/hera_tasks/tasks/resources/speech_recognition/questions_original.txt', 'r') as f:
            self.questions_original = f.read().split(';')
            self.questions_original = [q.strip() for q in self.questions_original]
            self.questions_original = list(filter(None,self.questions_original))

        with open('/home/robofei/catkin_hera/src/hera_robot/hera_tasks/tasks/resources/speech_recognition/answers_original.txt', 'r') as f:
            self.answers_original = f.read().split(';')
            self.answers_original = [a.strip() for a in self.answers_original]
            self.answers_original = list(filter(None, self.answers_original))

        # Task
        #actions.talk('Starting Speech Recognition Task')
        # Wait door opening to start
        #door = False
        #while not json.loads(actions.question('is_front_free').result):
            #if not door:
                #actions.talk('Waiting to open the door')
                #door = True
            #continue

        #actions.talk('The door is open!')
        #actions.move('foward', seconds=5.0)
        #actions.goto('riddles')
        #actions.talk('I am ready to start speech recognition task, please stand in front of me')
        # Repeat recognition 5 times

        for i in range(5):
            actions.talk('Please, make your question after the signal')
            try:
                # Start GSR request and listen to the question
                sr_question = StartRequest()
                 
                sr_question.spec = self.questions
                speech = actions.hear(sr_question)
                log_question=''
                # Get the recognized question index to get the answer
                if speech.result in self.questions:
                    answer = self.answers[self.questions.index(speech.result)]
                else:
                    answer = 'Sorry, I can\'t find an answer for that question.'

                # Answer the question
                actions.talk(answer)
                # for i in self.answers_original:
                #     #print(i)
                #     if answer==i:
                #         log_question = self.questions_original[self.answers_original.index(i)]
               
                # print(log_question)
                # print(answer)
                    

            except Exception as e:
                rospy.logerr(e.message)
                traceback.print_exc()
        # End Task
        actions.talk('Speech Recognition Task finished')


if __name__ == '__main__':
    try:
        rospy.init_node('speech_recognition')
        SpeechRecognition()

    except KeyboardInterrupt:
        pass
