import rospy
import json

import methods.General as General
import methods.Speech as Speech
import methods.Perception as Perception
import methods.Navigation as Navigation
import methods.Manipulator as Manipulator

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

        self.locals = ['table', 'cabinet']

    def task(self):
        """
        Main task of the robot.
        :return: None
        """
        object_result = False
        self.speech.talk("Starting Storing Groceries Task.")
        for i in range(5):
            self.navigation.goto(self.locals[0])
            self.speech.talk('Trying to pick object')
            while not object_result:
                obj_coordinates = self.perception.find_obj('closest')
                if obj_coordinates is not None:
                    manip_result = self.manipulator.send_goal(obj_coordinates, 'pick')
                    if manip_result == 'success':
                        object_result = True
                        self.speech.talk('Object picked')
                    else:
                        self.speech.talk('Object not picked, trying again')
                        self.manipulator.send_goal('open')
                        self.manipulator.send_goal('home')
                        self.navigation.move('spin_left', 0.3)
                        rospy.sleep(1)

        self.speech.talk('Going to cabinet')
        self.navigation.goto(self.locals[1])
        while not object_result:
            manip_result = self.manipulator.send_goal('place')
            if manip_result == 'success':
                object_result = True
                self.speech.talk('Object placed')
            else:
                self.speech.talk('Object not placed correctly, trying again')
