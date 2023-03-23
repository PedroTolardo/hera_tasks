from cmath import sin
import actionlib
import rospy

from hera.msg import poseFeedback, poseResult, poseAction, poseGoal
from hera.msg import gotoFeedback, gotoResult, gotoAction, gotoGoal
from hera.msg import gotosocialFeedback, gotosocialResult, gotosocialAction, gotosocialGoal
from hera.msg import gotoposeFeedback, gotoposeResult, gotoposeAction, gotoposeGoal
from hera.msg import talkFeedback, talkResult, talkAction, talkGoal
from hera.msg import hearFeedback, hearResult, hearAction, hearGoal
from hera.msg import faceFeedback, faceResult, faceAction, faceGoal
from hera.msg import moveFeedback, moveResult, moveAction, moveGoal
# from hera.msg import followFeedback, followResult, followAction, followGoal
from hera.msg import savelocalFeedback, savelocalResult, savelocalAction, savelocalGoal
from hera.msg import headFeedback, headResult, headAction, headGoal

from hera.srv import question

from std_msgs.msg import Float32

from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from tf2_ros import TransformException as Exception, ConnectivityException, LookupException, ExtrapolationException
import tf2_geometry_msgs
import tf
from tf import TransformListener

from hera_control.srv import Manip3
from hera_control.msg import Manip3_Goal

from dynamixel_workbench_msgs.srv import DynamixelCommand

from hera_control.srv import Head_service
from hera_control.msg import Head_Goal

from hera_objects.srv import FindObject, FindSpecificObject
from hera_objects.msg import *

# from people.srv import Once

# from social_worlds.srv import *
from map.srv import SaveLocal

from hera_face.srv import *

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

from pydub import AudioSegment
from pydub.playback import play


class Actions():
    """docstring for Actions."""

    def __init__(self, lang):

        self.lang = lang
        self.tf_listener = tf.TransformListener()

        self.odom = None
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('/base_scan_front', LaserScan, self.callback_laser)

        # init clients
        self.question = rospy.ServiceProxy('/question', question)
        self.manipulator = rospy.ServiceProxy('/manipulator', Manip3)
        self.dynamixel = rospy.ServiceProxy('/dynamixel_controller/dynamixel_command', DynamixelCommand)
        self.head_interface = rospy.ServiceProxy('/head_interface', Head_service)
        self.face_checking = rospy.ServiceProxy('/face_check', face_check)
        self.objects = rospy.ServiceProxy('/objects', FindObject)
        self.save_obj_local = rospy.ServiceProxy('/map/save_local', SaveLocal)
        self.save_face = rospy.ServiceProxy('/face_captures', face_capture)
        # self.detected = rospy.ServiceProxy('/detect_output', detect_output)
        self.recog_face = rospy.ServiceProxy('/face_recog', face_list)
        # self.color_filter = rospy.ServiceProxy('/color_filter', color_detect)
        self.specific_object = rospy.ServiceProxy('/specific_object', FindSpecificObject)
        # self.people = rospy.ServiceProxy('/people/once', Once)
        self.client_pose = actionlib.SimpleActionClient('/pose', poseAction)
        self.client_goto = actionlib.SimpleActionClient('/goto', gotoAction)
        self.client_gotosocial = actionlib.SimpleActionClient('/gotosocial', gotosocialAction)
        self.client_gotopose = actionlib.SimpleActionClient('/gotopose', gotoposeAction)
        self.client_talk = actionlib.SimpleActionClient('/talk', talkAction)
        self.client_hear = actionlib.SimpleActionClient('/hear', hearAction)
        self.client_face = actionlib.SimpleActionClient('/face', faceAction)
        self.client_move = actionlib.SimpleActionClient('/move', moveAction)
        # self.client_follow = actionlib.SimpleActionClient('/follow', followAction)
        self.client_savelocal = actionlib.SimpleActionClient('/savelocal', savelocalAction)
        self.client_head = actionlib.SimpleActionClient('/head', headAction)

        # wait for servers
        print('wait for server: question')
        rospy.wait_for_service('/question')
        print('wait for server: manipulator')
        # rospy.wait_for_service('/manipulator')
        print('wait for server: objects')
        # rospy.wait_for_service('/objects')
        print('wait for server: specific_object')
        # rospy.wait_for_service('/specific_object')
        # print ('wait for server: people')
        # # rospy.wait_for_service('/people/once')

        print('wait for server: pose')
        self.client_pose.wait_for_server()
        print('wait for server: goto')
        self.client_goto.wait_for_server()
        print('wait for server: gotosocial')
        self.client_gotosocial.wait_for_server()
        print('wait for server: gotopose')
        self.client_gotopose.wait_for_server()
        print('wait for server: talk')
        self.client_talk.wait_for_server()
        print('wait for server: hear')
        self.client_hear.wait_for_server()
        print('wait for server: face')
        self.client_face.wait_for_server()
        print('wait for server: move')
        self.client_move.wait_for_server()
        # print 'wait for server: follow')
        # self.client_follow.wait_for_server()
        print('wait for server: savelocal')
        self.client_savelocal.wait_for_server()
        print('wait for server: head')
        # self.client_head.wait_for_server()

    def callback_odom(self, msg):
        self.odom = msg.pose.pose

    def callback_laser(self, msg):
        self.laser_ran = (msg.ranges)

    def question(q):
        resp = self.question(q)
        return resp.result

    def savelocal(self, local):
        goal = savelocalGoal(location=local)
        self.client_savelocal.send_goal(goal)
        self.client_savelocal.wait_for_result()
        return self.client_savelocal.get_result()

    def transform_pose(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            # transform = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Duration(1))
            # output_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

    def save_obj_location(self, name, obj, dist):
        for i in range(2):
            listener = TransformListener()
            rospy.sleep(5)
            t = rospy.Time(0)

            pose = Pose()
            # pegando orientacao do robo
            (robot_trans, robot_rot) = self.tf_listener.lookupTransform('/map', '/base', rospy.Time(0))
            # Relacao Obj -> map (com a mesma orientacao do robo)
            # --------------------------------------
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.position.z
            pose.orientation.x = robot_rot[0]
            pose.orientation.y = robot_rot[1]
            pose.orientation.z = robot_rot[2]
            pose.orientation.w = robot_rot[3]
            self.save_obj_local("pose_1", pose)

            pose_2 = Pose()
            # tf Obj -> robo
            (obj_trans, obj_rot) = self.tf_listener.lookupTransform('/base', '/pose_1', rospy.Time(0))
            pose_2.position.x = obj_trans[0]
            pose_2.position.y = obj_trans[1]
            pose_2.position.z = obj_trans[2]
            pose_2.orientation.x = obj_rot[0]
            pose_2.orientation.y = obj_rot[1]
            pose_2.orientation.z = obj_rot[2]
            pose_2.orientation.w = obj_rot[3]

            print("Pose2_antes:", pose_2)
            # retirando a distancia
            pose_2.position.x = pose_2.position.x - dist
            pose_2.position.y = pose_2.position.y + 0.1

            # --------------------------------------

            print("Pose2_dps:", pose_2)
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose_2
            pose_stamped.header.frame_id = '/base'
            pose_stamped.header.stamp = rospy.Time(0)

            listener.waitForTransform("/map", "/base", t, rospy.Duration(5))

            if listener.canTransform("/map", "/base", t):
                pose_final = listener.transformPose("/map", pose_stamped)
                print("Pose_final_stamped:", pose_final)

            else:
                rospy.logerr('Transformation is not possible!')
                sys.exit(0)
            # pose_final.position.x = transformed_pose[0] 
            # pose_final.position.y = transformed_pose[1]
            # pose_final.position.z = transformed_pose[2]
            # pose_final.orientation.x = transformed_rot[0]
            # pose_final.orientation.y = transformed_rot[1]
            # pose_final.orientation.z = transformed_rot[2]
            # pose_final.orientation.w = transformed_rot[3]

            # self.save_obj_local("pose_final", pose_final)
            # -------------------------------------
            ##Relacao pose_final -> map (com a mesma orientacao do robo)

        pose_final_2 = Pose()
        # (final_trans, final_rot) = self.tf_listener.lookupTransform('/map', "/pose_final", rospy.Time(0))
        pose_final_2.position.x = pose_final.pose.position.x
        pose_final_2.position.y = pose_final.pose.position.y
        pose_final_2.position.z = pose_final.pose.position.z
        pose_final_2.orientation.x = robot_rot[0]
        pose_final_2.orientation.y = robot_rot[1]
        pose_final_2.orientation.z = robot_rot[2]
        pose_final_2.orientation.w = robot_rot[3]
        print("Pose_final:", pose_final_2)

        resp = self.save_obj_local(name, pose_final_2)
        print("resp: ", resp)
        return True  # if resp.result == 'SUCCEDED' else False

    # def save_obj_location(self, name, obj):
    #     pose = Pose()
    #     (target_trans, target_rot) = self.tf_listener.lookupTransform('/map', '/base', rospy.Time(0))        
    #     pose.position.x = obj.position.x 
    #     pose.position.y = obj.position.y
    #     pose.position.z = obj.position.z
    #     pose.orientation.x = target_rot[0]
    #     pose.orientation.y = target_rot[1]
    #     pose.orientation.z = target_rot[2]
    #     pose.orientation.w = target_rot[3]

    #     resp = self.save_obj_local(name, pose)
    #     #print("resp: ",resp)
    #     return True #if resp.result == 'SUCCEDED' else False

    def manip(self, tipo="", x=0.0, y=0, z=0, rx=0, ry=0, rz=0):
        m3 = Manip3_Goal()
        m3.x = x
        m3.y = y
        m3.z = z
        m3.rx = rx
        m3.ry = ry
        m3.rz = rz
        success = self.manipulator(tipo, m3)
        return success.result

    def manip_goal(self, goal, tipo=""):
        m3 = Manip3_Goal()
        m3.x = goal.position.x
        m3.y = goal.position.y
        m3.z = goal.position.z
        m3.rx = goal.position.rx
        m3.ry = goal.position.ry
        m3.rz = goal.position.rz
        success = self.manipulator(tipo, m3)
        return success.result

    def dynamixel_goal(self, id, position):
        self.dynamixel('', id, 'Goal_Position', position)
        return True

    def find_obj(self, condition=""):
        resp = self.objects(condition)
        values = [resp.position.x, resp.position.y, resp.position.z]
        if all(v == 0.0 for v in values):
            return None
        else:
            return resp

    def FindSpecificObject(self, tipo=""):
        resp = self.specific_object(tipo)
        values = [resp.position.x, resp.position.y, resp.position.z]
        if all(v == 0.0 for v in values):
            return None
        else:
            return resp

    # def FindSpecificObject(self, tipo=""):
    #     resp = self.specific_object(tipo)
    #     values = [resp.position.x, resp.position.y, resp.position.z]
    #     if all(v == 0.0 for v in values):
    #         return None
    #     else:
    #         return resp.position

    def head(self, tipo="", tilt=0.0):
        h = Head_Goal()
        h.z = tilt
        resp = self.head_interface(tipo, h)
        return True if resp.result == 'SUCCEEDED' else False

    def pose(self, location):
        goal = poseGoal(location=location)
        self.client_pose.send_goal(goal)
        self.client_pose.wait_for_result()
        return self.client_pose.get_result()

    def people(self):
        resp = self.people()
        return resp.people

    def goto(self, location, wait=True):
        if location == None:
            self.client_goto.cancel_all_goals()
            return
        goal = gotoGoal(location=location)
        self.client_goto.send_goal(goal)
        if wait:
            self.client_goto.wait_for_result()
            return self.client_goto.get_result()

    def goto_multiple(self, locations, wait=True):
        reached_locations = []
        if locations is None:
            self.client_goto.cancel_all_goals()
            return

        for i in range(len(locations)):
            self.goto(locations[i], wait)
            reached_locations.append(locations[i])
        return reached_locations

    def gotosocial(self, location, wait=True):
        if location == None:
            self.client_gotosocial.cancel_all_goals()
            return
        goal = gotosocialGoal(location=location)
        self.client_gotosocial.send_goal(goal)
        if wait:
            self.client_gotosocial.wait_for_result()
            return self.client_gotosocial.get_result()

    def gotopose(self, location, frame, wait=True):
        if location == None:
            self.client_goto.cancel_all_goals()
            return
        goal = gotoGoal(location=location, reference=frame)
        self.client_goto.send_goal(goal)
        if wait:
            self.client_goto.wait_for_result()
            return self.client_goto.get_result()

    def talk(self, phrase, from_lang="en", to_lang=None):
        if (not to_lang): to_lang = self.lang
        goal = talkGoal(phrase=phrase, from_lang=from_lang, to_lang=to_lang)
        self.client_talk.send_goal(goal)
        self.client_talk.wait_for_result()
        return self.client_talk.get_result()

    def hear(self, srv):
        goal = hearGoal(spec=srv.spec, choices=srv.choices)
        self.client_hear.send_goal(goal)
        self.client_hear.wait_for_result()
        return self.client_hear.get_result()

    def face(self):
        goal = faceGoal()
        self.client_face.send_goal(goal)
        self.client_face.wait_for_result()
        return self.client_face.get_result()

    def save_face(self, request):  # adicionar nome
        resp = self.save_face(request)
        return str(resp)

    def obj_output(self, request):  # adicionar nome
        resp = self.detected(request)
        return str(resp)

    def face_recog(self, request):
        resp = self.recog_face(request)
        print("resp: ", resp.result)
        print("center: ", resp.center)
        print("num: ", resp.num)

        return resp.result, resp.center, resp.num

    def face_check(self):
        resp = self.face_checking('')
        # retornar somente o resultado em booleano
        return resp.result
        # print("resp: ", resp)
        # return resp

    # def face_move_recog(self, request, rep):
    #    for i in range(rep):
    #        resp = self.recog_face(request)
    #
    #        if resp.result == '':
    #            self.move('foward', 0.1, 0.1)
    #            self.move('stop', 0, 0)

    #         else:
    #             break

    #     print("resp: ", resp.result)
    #     print("center: ", resp.center)

    #     return resp.result, resp.center

    def lugar_vazio(self, guest1, guest2, guest3):
        num_lugares = 5
        seat1 = False
        seat2 = True
        seat3 = False
        seat4 = False
        seat5 = True

        seat1_point = 640 / (2 * num_lugares)
        seat2_point = 640 * 3 / (2 * num_lugares)
        seat3_point = 640 * 5 / (2 * num_lugares)
        seat4_point = 640 * 7 / (2 * num_lugares)

        if 0 <= guest1 < (1 / num_lugares * 640) or 0 <= guest2 < (1 / num_lugares * 640) or 0 <= guest3 < (
                1 / num_lugares * 640):
            seat1 = True

        if (2 / num_lugares * 640) < guest1 < (3 / num_lugares * 640) or (2 / num_lugares * 640) < guest2 < (
                3 / num_lugares * 640) or (2 / num_lugares * 640) < guest3 < (3 / num_lugares * 640):
            seat3 = True

        if (3 / num_lugares * 640) < guest1 < (4 / num_lugares * 640) or (3 / num_lugares * 640) < guest2 < (
                4 / num_lugares * 640) or (3 / num_lugares * 640) < guest3 < (4 / num_lugares * 640):
            seat4 = True

        print(seat1)
        print("1", guest1)
        print(guest2)
        print(guest3)

        if seat1 == False:
            return (seat1_point)
        elif seat1 == True and seat3 == False:
            return (seat3_point)
        elif seat1 == True and seat3 == True and seat4 == False:
            return (seat4_point)
        else:
            return None

    # def color_filter(self, request):
    #     resp = self.recog_face(request)
    #     print("center x: ", resp.center_x)
    #     print("center y: ", resp.center_y)

    #     return resp.center_x, resp.center_y 

    def center_filter(self, vel, time):
        check_x = False
        check_y = False
        check_pose = False
        while not check_pose:
            if not check_y:
                resp = self.color_filter('')
                if resp.center_y < 246.5:  # 158.5:
                    self.move('foward', vel, time)
                    self.move('stop', 0, 0)
                elif resp.center_y > 266.6:  # 178.5:
                    self.move('backward', vel, time)
                    self.move('stop', 0, 0)
                else:
                    check_y = True
            if not check_x:
                resp = self.color_filter('')
                if resp.center_x < 312.5:  # 291.0:
                    self.move('right', vel, time)
                    self.move('stop', 0, 0)
                elif resp.center_x > 332.5:  # 311.0:
                    self.move('left', vel, time)
                    self.move('stop', 0, 0)
                else:
                    check_x = True
                    check_y = False
            else:
                check_pose = True
        return True

    def laser_line(self, dist):
        rospy.wait_for_message('/base_scan_front', LaserScan)
        # rospy.loginfo(self.laser_ran)
        aux = 0
        while (aux == 0):
            laser_ran = self.laser_ran
            amp_ang = 166  # Amplitude de leitura do laser(graus)
            range_size = len(laser_ran)  # Numero de medidas lidas pelo laser
            lim_ang = 15  # Angulo para limitar a leitura
            lim_laser = round(lim_ang * range_size / amp_ang)
            start = laser_ran[int((range_size / 2) - (lim_laser / 2))]  # left
            end = laser_ran[int((range_size / 2) + (lim_laser / 2))]  # right
            diff = start - end
            if (diff > 0.001):
                self.move('spin_left')
            elif (diff < -0.001):
                self.move('spin_right')
            else:
                self.move('stop')
                # print("alinhado")
                aux = 1
        dist_robo = laser_ran[int(len(laser_ran) / 2)]
        if (dist_robo > dist):
            while (dist_robo > dist):
                laser_ran = self.laser_ran
                dist_robo = laser_ran[int(len(laser_ran) / 2)]
                self.move('foward')
                # self.move('backward')
        elif (dist_robo < dist):
            while (dist_robo < dist):
                laser_ran = self.laser_ran
                dist_robo = laser_ran[int(len(laser_ran) / 2)]
                # self.move('foward')
                self.move('backward')
        self.move('stop')

    def recog_garbage(self, range, dist):
        rospy.wait_for_message('/base_scan_front', LaserScan)
        # rospy.loginfo(self.laser_ran)
        enter_bin = 0
        while enter_bin != 3:
            laser_ran = self.laser_ran
            amp_ang = 166  # Amplitude de leitura do laser(graus)
            range_size = len(laser_ran)  # Numero de medidas lidas pelo laser
            lim_ang = 60  # Angulo para limitar a leitura
            lim_laser = round(lim_ang * range_size / amp_ang)
            filter_range = laser_ran[int((range_size / 2) - (lim_laser / 2)):int(
                (range_size / 2) + (lim_laser / 2))]  # Leitura limitada
            center_bin = filter_range.index(min(filter_range))
            center_range = filter_range.index(filter_range[int(len(filter_range) / 2)])
            # rospy.loginfo(center_bin)
            # rospy.loginfo(center_range)

            if (center_range > center_bin + (range / 2)):
                # rospy.loginfo("vai pra direita")
                enter_bin = 2
                self.move('spin_right')  # inverter


            elif (center_range < center_bin - (range / 2)):
                # rospy.loginfo("vai pra esquerda")
                enter_bin = 1
                self.move('spin_left')
            else:
                # rospy.loginfo("chegou")
                enter_bin = 3
                self.move('stop')
                pass
        dist_robo = laser_ran[int(len(laser_ran) / 2)]
        if (dist_robo > dist):
            while (dist_robo > dist):
                laser_ran = self.laser_ran
                dist_robo = laser_ran[int(len(laser_ran) / 2)]
                self.move('foward', 0.1)
                # self.move('backward')
        elif (dist_robo < dist):
            while (dist_robo < dist):
                laser_ran = self.laser_ran
                dist_robo = laser_ran[int(len(laser_ran) / 2)]
                # self.move('foward')
                self.move('backward', 0.1)
        self.move('stop')

    def click(self):
        wav_file = AudioSegment.from_file(file="/home/robofei/catkin_hera/src/3rdParty/vision_system/audio.wav",
                                          format="wav")
        play(wav_file)

    def beep(self):
        wav_file = AudioSegment.from_file(
            file="/home/robofei/catkin_hera/src/3rdParty/speech_recognition/gsr_ros/beep.ogg")
        play(wav_file)

    def move(self, cmd, vel=0.2, seconds=0.0):
        goal = moveGoal(cmd=cmd, vel=vel, seconds=seconds)
        self.client_move.send_goal(goal)
        self.client_move.wait_for_result()
        return self.client_move.get_result()

    # def follow(self, location, wait=True):
    #    if location==None:
    #        self.client_follow.cancel_all_goals()
    #        return
#
#    goal = followGoal(location=location)
#    self.client_follow.send_goal(goal)
#    if wait:
#        self.client_follow.wait_for_result()
#        return self.client_follow.get_result()
