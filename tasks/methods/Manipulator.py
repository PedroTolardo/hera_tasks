import actionlib
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from hera_control.msg import Head_Goal, Manip_Goal
from hera_control.srv import Head, Manip
from std_msgs.msg import Float32


class Manipulator:
    """
    A class that represents the manipulator of the HERA robot.

    Attributes
    ----------
    manipulator_service : rospy.ServiceProxy
        The service proxy to the manipulator of the HERA robot.
    dynamixel : rospy.ServiceProxy
        The service proxy to the Dynamixel controller of the HERA robot.
    head_service : rospy.ServiceProxy
        The service proxy to the head of the HERA robot.

    Methods
    -------
    __init__(self, lang)
        Initialize the Manipulator object by creating the service proxies and waiting for the servers.
    send_goal(self, coordinates, type="")
        Send the goal coordinates of the manipulator to the server.
    dynamixel_goal(self, id, position)
        Send the goal position to the Dynamixel controller.
    head(self, type="", tilt=0.0)
        Move the head of the HERA robot to the desired position.
    """

    def __init__(self, lang):
        # init clients
        self.manipulator_service = rospy.ServiceProxy('/manipulator', Manip)
        self.dynamixel = rospy.ServiceProxy('/dynamixel_controller/dynamixel_command', DynamixelCommand)
        self.head_service = rospy.ServiceProxy('/head_service', Head)

        # wait for servers
        rospy.loginfo('Waiting for server: manipulator')
        rospy.wait_for_service('/manipulator')
        rospy.loginfo('Waiting for server: dynamixel_controller/dynamixel_command')
        rospy.wait_for_service('/dynamixel_controller/dynamixel_command')
        rospy.loginfo('Waiting for server: head_service')
        rospy.wait_for_service('/head_service')

    def send_goal(self, coordinates, type=""):
        manipulator = Manip_Goal()
        manipulator.x = coordinates.position.x
        manipulator.y = coordinates.position.y
        manipulator.z = coordinates.position.z
        manipulator.rx = coordinates.position.rx
        manipulator.ry = coordinates.position.ry
        manipulator.rz = coordinates.position.rz
        success = self.manipulator_service(type, manipulator)
        return success

    def dynamixel_goal(self, id, position):
        self.dynamixel('', id, 'Goal_Position', position)
        return True

    def head(self, type="", tilt=0.0):
        head = Head_Goal()
        head.z = tilt
        success = self.head_service(type, head)
        return success
