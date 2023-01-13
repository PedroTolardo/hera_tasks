#!/usr/bin/env python3
from Actions import Actions
from std_msgs.msg import UInt32
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest

from hera_control.srv import Manip3
from hera_control.msg import Manip3_Goal

import rospy
import json


class Manipulation:
    """Manipulation Task"""

    def __init__(self):
        lang = rospy.get_param('~lang', 'en')

        actions = Actions(lang)

        # Task
        #actions.talk('Starting Manipulation and object detection Task')

        #Wait door opening to start
        #door = False
        #while not json.loads(actions.question('is_front_free').result):
        #    if not door:
        #        actions.talk('Knock knock')
        #        door = True
        #    continue
        #actions.talk('The door is open!')
        #actions.move('foward', seconds=5.0)

        # Manipulation Step
        actions.head("", 3.0)
        actions.manip('open')
        actions.manip('home')
        actions.laser_line(0.3)
        #actions.laser_line(0.5)
        actions.talk('Saving Objects')
        actions.head("", 3.1)
        #actions.obj_output('')
        actions.talk('Starting manipulation')
        for i in range(5):
            # Repeat 5 times
            # Go to first shelf and alignß
            if i != 0:
                actions.talk('Going to shelf 1')
                actions.goto('shelf1')
                actions.laser_line(0.3)

            # Set up manipulator and head to detect objects
            actions.manip('open')
            actions.manip('home')
            #actions.head("", 3.2)
            # Get closest object
            manip_success = False
            while not manip_success:
                self.coordinates = actions.find_obj('closest')
                if self.coordinates is not None:
                    ## TODO: Say the object that is going to be picked
                    success = actions.manip_goal(self.coordinates, 'pick2')
                    if success == "success":
                        actions.talk('Pick succeded')
                        manip_success = True
                    else:
                        actions.talk('Pick failed')
                        actions.manip('open')
                        actions.manip('home')
                        actions.talk('Trying again.')
                        actions.move('spin_left', 0.3)
                        rospy.sleep(1)
                        continue

            # Go to second shelf and align
            actions.talk('Going to shelf 2')
            actions.goto('shelf2')
            actions.laser_line(0.2)

            #Place object
            manip_success = False
            while not manip_success:
                success = actions.manip('place3', float(i))
                if success == "success":
                    actions.talk('Place succeded')
                    rospy.sleep(3)
                    manip_success = True
                else:
                    actions.talk('Place failed. Trying again')
                    continue

        actions.talk('Manipulation task finished')



if __name__ == '__main__':
    try:
        rospy.init_node('manipulation')
        Manipulation()

    except KeyboardInterrupt:
        pass
