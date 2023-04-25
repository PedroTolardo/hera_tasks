#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import methods.Navigation as Navigation


class Task():
    def __init__(self):
        self.navigation = Navigation.Navigation()

        # task variables
        self.locals = ['point_1', 'point_2', 'point_3']

    def run(self):
        for i in range(len(self.locals)):
            rospy.loginfo('Going to ' + self.locals[i])
            self.navigation.goto(self.locals[i])


if __name__ == '__main__':
    rospy.init_node('Task_Navigation', anonymous=True)
    task = Task()
    task.run()
    rospy.spin()