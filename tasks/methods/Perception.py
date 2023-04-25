import rospy
import actionlib
import sys
import tf
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped

from hera.msg import faceFeedback, faceResult, faceAction, faceGoal
from hera_objects.srv import Findobject, FindSpecificobject
from hera_objects.msg import *
from hera_face.srv import *


class Perception:
    def __init__(self):
        self.tf_listener = tf.TransformListener()

        self.face_checking = rospy.ServiceProxy('/face_check', face_check)
        self.save_face = rospy.ServiceProxy('/face_captures', face_capture)
        self.recog_face = rospy.ServiceProxy('/face_recog', face_list)
        self.objects = rospy.ServiceProxy('/objects', FindObject)
        self.color_filter = rospy.ServiceProxy('/color_filter', color_detect)
        self.specific_object = rospy.ServiceProxy('/specific_object', FindSpecificObject)

        self.client_face = actionlib.SimpleActionClient('/face', faceAction)

        rospy.loginfo('Waiting for server: objects')
        rospy.wait_for_service('/objects')
        rospy.loginfo('Waiting for server: specific_object')
        rospy.wait_for_service('/specific_object')

        rospy.loginfo('Waiting for server: face')
        self.client_face.wait_for_server()

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

    def people(self):
        answer = self.people()
        return answer.people

    def save_face(self, request):
        answer = self.save_face(request)
        return str(answer)

    def face_recog(self, request):
        resp = self.recog_face(request)
        rospy.loginfo('Resp: ', resp.result)
        rospy.loginfo('Center: ', resp.center)
        rospy.loginfo('Num: ', resp.num)
        return resp.result, resp.center, resp.num

    def obj_output(self, request):
        resp = self.detected(request)
        return str(resp)

    def face_check(self):
        resp = self.face_checking('')
        return resp.result