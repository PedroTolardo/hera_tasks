#!/usr/bin/env python3

import traceback
import rospy
import math
import json
import time

from std_msgs.msg import UInt32
from gsr_ros.msg import Opcs
from gsr_ros.srv import StartRequest
from navigation_methods import Navigation

from Actions import Actions


class Task():
    """docstring for Task"""

    def __init__(self):
        nav = Navigation()
        actions = Actions(lang='en')

        # task variables
        robotnames = ['Robot', 'Hera']
        known_places = json.loads(actions.question('know_places').result)
        specs = [
            '<robotnames>',
            'go to <known_places>',
        ]

        # speech variables
        self.srv = StartRequest()
        self.srv.spec = specs
        self.srv.choices.append(Opcs(id="robotnames", values=robotnames))
        self.srv.choices.append(Opcs(id="known_places", values=known_places))

        ########################################################################
        ########################################################################

        while True:
            result = nav.goto_multiple(known_places, wait=True)
            rospy.loginfo(result)


if __name__ == '__main__':
    try:
        rospy.init_node('Task')
        Task()
    except KeyboardInterrupt:
        pass
