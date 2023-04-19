from cmath import sin
import actionlib
import rospy
from hera.msg import (poseFeedback, poseResult, poseAction, poseGoal,
                      gotoFeedback, gotoResult, gotoAction, gotoGoal,
                      gotosocialFeedback, gotosocialResult, gotosocialAction, gotosocialGoal,
                      gotoposeFeedback, gotoposeResult, gotoposeAction, gotoposeGoal,
                      moveFeedback, moveResult, moveAction, moveGoal,
                      savelocalFeedback, savelocalResult, savelocalAction, savelocalGoal)
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
import tf
from tf import TransformListener
from map.srv import SaveLocal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from typing import Any, Optional, Union


class Navigation:
    """
    A class that contains the navigation methods of the HERA robot.

    Attributes
    ----------
    tf_listener : tf.TransformListener
        The transform listener to get the transform between the frames.
    odom : Pose
        The current pose of the robot.
    laser_ranges : list
        The current laser scan ranges.

    Methods
    -------
    __init__(self)
        Initialize the Navigation object by creating the service proxies and waiting for the servers.
    callback_odom(self, msg)
        Callback function for the odom subscriber.
    callback_laser(self, msg)
        Callback function for the laser subscriber.
    save_local(self, location)
        Saves the current local map to the specified location.
    goto(self, location, wait)
        Moves the robot to the specified location.
    goto_social(self, x, y, theta)
        Moves the robot to the specified pose.
    goto_pose(self, x, y, theta)
        Moves the robot to the specified pose.
    move(self, x, y, theta)
        Moves the robot to the specified pose.
    pose(self, x, y, theta)
        Moves the robot to the specified pose.
    align_with_object(self, range, max_distance, limit_angle)
        Aligns the robot with the object in front of it.


    """

    def __init__(self) -> None:

        self.tf_buffer: tf2_ros.Buffer = tf2_ros.Buffer()
        self.tf_listener: tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom: Odometry = None
        self.laser_readings: LaserScan = None

        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('/base_scan_front', LaserScan, self.callback_laser)

        # init clients
        self.client_pose: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/pose', poseAction)
        self.client_goto: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/goto', gotoAction)
        self.client_gotosocial: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/gotosocial', gotosocialAction)
        self.client_gotopose: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/gotopose', gotoposeAction)
        self.client_move: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/move', moveAction)
        self.client_savelocal: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/savelocal', savelocalAction)

        # wait for servers
        rospy.loginfo('Waiting for server: pose')
        self.client_pose.wait_for_server()
        rospy.loginfo('Waiting for server: goto')
        self.client_goto.wait_for_server()
        rospy.loginfo('Waiting for server: gotosocial')
        self.client_gotosocial.wait_for_server()
        rospy.loginfo('Waiting for server: gotopose')
        self.client_gotopose.wait_for_server()
        rospy.loginfo('Waiting for server: move')
        self.client_move.wait_for_server()
        rospy.loginfo('Waiting for server: savelocal')
        self.client_savelocal.wait_for_server()

    def callback_odom(self, msg: Odometry) -> None:
        self.odom = msg.pose.pose

    def callback_laser(self, msg: LaserScan) -> None:
        self.laser_readings = msg.ranges

    def save_local(self, location: str) -> Any:
        """
        Saves the current local map to the specified location.

        :param location: The location to save the map to.
        :return: None if there is no result, otherwise a string indicating the result of the movement.
        """
        goal = savelocalGoal(location=location)
        self.client_savelocal.send_goal(goal)
        self.client_savelocal.wait_for_result()
        return self.client_savelocal.get_result()

    @staticmethod
    def transform_pose(input_pose: Pose, from_frame: str, to_frame: str) -> Optional[Pose]:
        """
        Transform a pose from one frame to another.

        :param input_pose: The pose to transform.
        :param from_frame: The frame of the input pose.
        :param to_frame: The frame to transform the pose to.
        :return: The transformed pose.
        """

        pose_stamped = tf2_geometry_msgs.PoseStamped(
            header=tf2_geometry_msgs.msg.Header(
                frame_id=from_frame,
                stamp=rospy.Time.now()),
            pose=input_pose)

        try:
            output_pose_stamped = self.tf_buffer.transform(
                pose_stamped,
                to_frame,
                rospy.Duration(1.0))

            return output_pose_stamped.pose

        except tf2_ros.TransformException as e:
            rospy.logerr(str(e))
            return None

    def pose(self, location: str) -> Any:
        """
        Moves the robot to the specified location in the map frame.

        :param location: The location to move to.
        :return: None if there is no result, otherwise a string indicating the result of the movement.
        """
        goal = poseGoal(location=location)
        self.client_pose.send_goal(goal)
        self.client_pose.wait_for_result()
        return self.client_pose.get_result()

    def goto(self, location: str, wait: bool = True) -> Optional[str]:
        """
        Moves the robot to the specified location in the map frame.

        :param location: The location to move to.
        :param wait: Whether to wait for the movement to finish.
        :return: None if there is no result, otherwise a string indicating the result of the movement.
        """
        if location is None:
            self.client_goto.cancel_all_goals()
            return

        goal = gotoGoal(location=location)
        self.client_goto.send_goal(goal)

        if wait:
            self.client_goto.wait_for_result()
            return self.client_goto.get_result()

    def goto_multiple(self, locations: list, wait: bool = True) -> Optional[dict]:
        """
        Moves the robot to the specified location in the map frame.

        :param locations: The location list to move to
        :param wait: Whether to wait for the movement to finish.
        :return: None if there is no result, otherwise a string indicating the result of the movement.
        """
        results = []

        if locations is None:
            self.client_goto.cancel_all_goals()
            return

        for location in locations:
            rospy.loginfo("Going to location: " + location)
            goal = gotoGoal(location=location)
            self.client_goto.send_goal(goal)
            if wait:
                self.client_goto.wait_for_result()
                results.append(self.client_goto.get_result())

        result_dict = dict(zip(locations, results))
        return result_dict

    def goto_social(self, location, wait=True) -> Optional[str]:
        """
        Moves the robot to the specified location in the social frame.

        :param location: The location to move to.
        :param wait: Whether to wait for the movement to finish.
        :return: None if there is no result, otherwise a string indicating the result of the movement.
        """
        if location is None:
            self.client_gotosocial.cancel_all_goals()
            return
        goal = gotosocialGoal(location=location)
        self.client_gotosocial.send_goal(goal)
        if wait:
            self.client_gotosocial.wait_for_result()
            return self.client_gotosocial.get_result()

    def goto_pose(self, location: Optional[str], frame: str, wait: bool = True) -> Optional[str]:
        """
        Moves the robot to the specified location in the specified frame.

        :param location: The location to move to.
        :param frame: The frame of the location.
        :param wait: Whether to wait for the movement to finish.
        :return: None if there is no result, otherwise a string indicating the result of the movement.
        """
        if location is None:
            self.client_goto.cancel_all_goals()
            return None
        goal = gotoGoal(location=location, reference=frame)
        self.client_goto.send_goal(goal)
        if wait:
            self.client_goto.wait_for_result()
            return self.client_goto.get_result()

    def move(self, command: str, velocity: float = 0.2, duration: float = 0.0) -> Union[None, bool]:
        """
        Moves the robot according to the specified command, velocity, and duration.

        :param command: The command to execute.
        :param velocity: The velocity to use for the movement.
        :param duration: The duration to perform the movement.
        :return: None if there is no result, otherwise a boolean indicating whether the movement succeeded or not.
        """
        goal = moveGoal(cmd=command, vel=velocity, seconds=duration)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
        return self.move_client.get_result()

    def align_with_object(self, range: float, max_distance: float, limit_angle: int) -> None:
        """
        Aligns the robot with an object in front of it.

        :param range: The range of the object to align with.
        :param max_distance: The maximum distance to align with the object.
        :param limit_angle: The angle to limit the laser readings.
        :return: None
        """
        rospy.wait_for_message('/base_scan_front', LaserScan)

        object_position: int = 0
        filtered_range: List[float] = []
        while object_position != 3:

            laser_readings: List[float] = self.laser_readings
            amplitude_angle: int = 166  # Amplitude of the laser reading (degrees)
            range_size: int = len(laser_readings)  # Number of measurements read by the laser
            limit_laser: int = round(limit_angle * range_size / amplitude_angle)
            filtered_range: List[float] = laser_readings[int((range_size / 2) - (limit_laser / 2)):
                                                         int((range_size / 2) + (limit_laser / 2))]  # Limited reading

            center_object = filtered_range.index(min(filtered_range))
            center_range = filtered_range.index(filtered_range[int(len(filtered_range) / 2)])

            if center_range > (center_object + (range / 2)):
                object_position = 2
                self.move('spin_right', 0.2, 0.5)

            elif center_range < (center_object - (range / 2)):
                object_position = 1
                self.move('spin_left', 0.2, 0.5)

            else:
                object_position = 3
                self.move('stop', 0.0, 0.0)

        robot_distance = filtered_range[int(len(filtered_range) / 2)]

        while robot_distance > max_distance:
            self.move('forward', 0.2, 0.0)
            rospy.sleep(0.1)
            robot_distance = filtered_range[int(len(filtered_range) / 2)]

        while robot_distance < max_distance:
            self.move('backward', 0.2, 0.0)
            rospy.sleep(0.1)
            robot_distance = filtered_range[int(len(filtered_range) / 2)]
        self.move('stop', 0.0, 0.0)
