import actionlib
import rospy
import rospkg
from pydub import AudioSegment
from pydub.playback import play
from hera.msg import (hearFeedback, hearResult, hearAction, hearGoal,
                      talkFeedback, talkResult, talkAction, talkGoal)


class Speech:
    """
    A class that contains the speech methods of the HERA robot

    Attributes
    ----------
    client_talk : actionlib.SimpleActionClient
        A client that talks to the talk service
    client_hear : actionlib.SimpleActionClient
        A client that listens to the hear service

    Methods
    -------
    talk(phrase: str, from_language: str = 'en', to_language: str = 'en') -> talkResult
        A method that speaks a phrase

    hear(srv: List) -> hearResult
        A method that listens and verify the answer to a service

    """

    def __init__(self):
        self.client_talk = actionlib.SimpleActionClient('/talk', talkAction)
        self.client_hear = actionlib.SimpleActionClient('/hear', hearAction)

        rospy.loginfo('Waiting for server: hear')
        self.client_hear.wait_for_server()
        rospy.loginfo('Waiting for server: talk')
        self.client_talk.wait_for_server()

        rospack = rospkg.RosPack()

        def ask(self, question: question.Request) -> question.Response:
            return self.ask(question).result

        def talk(self, phrase: str, from_language: str = 'en', to_language: str = 'en') -> talkResult:
            """
            A method that speaks a phrase
            """
            goal = talkGoal(phrase=phrase, from_language=from_language, to_language=to_language)
            self.client_talk.send_goal(goal)
            self.client_talk.wait_for_result()
            return self.client_talk.get_result()

        def hear(self, srv: List) -> hearResult:
            """
            A method that listens to a service
            """
            goal = hearGoal(spec=srv.spec, choices=srv.choices)
            self.client_hear.send_goal(goal)
            self.client_hear.wait_for_result()
            return self.client_hear.get_result()

        def play_sound(self, sound: str):
            """
            A method that plays a sound
            """
            audio = AudioSegment.from_file(file=(str(rospack.getpath('speech_recognition') + '/sounds/' + sound + '.mp3')))
            play(audio)

        