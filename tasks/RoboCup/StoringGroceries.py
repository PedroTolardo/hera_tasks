import rospy
import json

import General
import Speech
import Perception
import methods.Navigation as Navigation
import Manipulator

from gsr_ros.srv import StartRequest
from gsr_ros.msg import Opcs


class StoringGroceries:
    """
    The robot stores groceries into a cabinet with shelves. Objects are sorted on the shelves based on similarity,
    for instance an apple is stored next to other fruits.
    """

    def __init__(self):

        rospy.init_node('StoringGroceries')

        self.speech = Speech.Speech()
        self.perception = Perception.Perception()
        self.navigation = Navigation.Navigation()
        self.manipulator = Manipulator.Manipulator()
        self.general = General.General()

        self.known_places = json.loads(self.general.question('know_places').result)

    def task(self):
        """
        Main task of the robot.
        :return: None
        """
        navigation.goto(table)

        object_coordinates = perception.find_obj('closest')

        manipulator.send_goal('attack')
        manipulator.send_goal(object_coordinates)

        navigation.goto(cabinet)
        navigation.align_with_object(0.3)









