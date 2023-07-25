#!/usr/bin/python
import scipy.io as spio
import os
import rospy
import tf
import time
import sys
import math
import string
import rospkg
from copy import deepcopy
from std_msgs.msg import Int32
from m3dp_msgs.srv import String
from m3dp_msgs.msg import TaskTrajectory, TaskPoint, PrintTrajectoryAction, PrintTrajectoryGoal, PrintTrajectoryFeedback, LayerProgress
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Joy
import actionlib
import struct

rospack = rospkg.RosPack()
m3dp_scenes_path = rospack.get_path("m3dp_scenes")


# Extruder
_retraction_rate = -1500
_print_rate = 1200 # rough guess for 4cm/s.
# _print_rate = 1500  # good for 6cm/s
# Trajectory
dt = .25  # 2.5 mm sampling means i dt is 0.25 s
speed_gain = 10  # speed will cause it to chase after carrot.3cm/s is good
tol_sat = 0.25
# z_ = 0.045
world_frame = "odom"
# world_frame = "ma_map"
# world_frame = "odom"
publish_real=True
# publish_real = True
layer = 0

xo = 0.0
yo = 0.0
zo = 0.12

# TASK_FILE = "/test_world/tasks/sine_25m_6cm_2layer_solved.mat"
# TASK_FILE = "/curved/tasks/th_1_solved2.mat"
# TASK_FILE = "/curved/tasks/th_2_solved2.mat"
# TASK_FILE = "/curved/tasks/th_3_solved2.mat"
# TASK_FILE = "/curved/tasks/floorplan_solved2.mat"
# TASK_FILE = "/pioneers/th_3_solved2.mat"
# TASK_FILE = "/pioneers/pioneers1_solved2.mat"
TASK_FILE = "/pioneers/pioneers2_solved.mat"




class Tasker:
    def __init__(self):
        # self.load_path_srv = rospy.Service(
        #     "~load_path", String, self.handle_load_path)
        self.diagnostics_traj_pub1 = rospy.Publisher("diagnostics/task_poses", Marker, queue_size=10, latch=True)
        self.diagnostics_traj_pub2 = rospy.Publisher("diagnostics/base_poses", PoseArray, queue_size=10, latch=True)
        self.diagnostics_traj_pub3 = rospy.Publisher("diagnostics/base_elipses", MarkerArray, queue_size=10, latch=True)
        self.diagnostics_traj_pub4 = rospy.Publisher("diagnostics/layer_process", LayerProgress, queue_size=10, latch=True)
        self.extruder_pub = rospy.Publisher("as_extruder/set_pulse_rate", Int32, queue_size=10, latch=True)
        self.joy_sub = rospy.Subscriber("/ui/joy", Joy, self.joy_cb)
        self.as_client = actionlib.SimpleActionClient('PrintTrajectory', PrintTrajectoryAction)
        self.print_rate = _print_rate
        self.prev_jb0 = 0
        self.prev_jb3 = 0
        self.as_client.wait_for_server()
        self.latest_progress=0
        rospy.sleep(2)
        rospy.loginfo("Print tasker started")
        self.send_print()
        rospy.loginfo("Print tasker exited")

    def send_print(self):

        vars = loadmat(m3dp_scenes_path+TASK_FILE)['scene_task']
        # print((vars["layers"]))
        for idx, layer in enumerate(vars["layers"]):             
            trajectory = self.layer2trajectory(layer,start_index=0)
            self.send_trajectory_diagnostics(trajectory)
            if publish_real:
                goal = PrintTrajectoryGoal()
                goal.trajectory = trajectory
                self.as_client.send_goal(goal, feedback_cb=self.print_feedbackcb, done_cb=self.print_donecb)
                self.as_client.wait_for_result()
                # wait it layer is done. if success continue
                result = self.as_client.get_result()
                if not result.success:
                    rospy.loginfo("Print not successfull tasker breaking.")
                    rospy.loginfo("Latest Completion: {}".format(self.latest_progress))
                    rospy.loginfo("Latest index: {}".format(math.floor(float(len(layer.task_positions))*self.latest_progress)))
                    
                    break
                raw_input("Press Enter to continue to next layer...")
            else:
                break

    def print_feedbackcb(self, feedback):
        # lp = LayerProgress()
        # lp.mpc_rate = feedback.mpc_rate
        # lp.tracker_rate = feedback.tracker_rate
        # lp.tf_rate = feedback.tf_rate
        # lp.obs_rate = feedback.obs_rate
        # lp.completion = feedback.completion
        # lp.start_time = feedback.start_time
        # lp.end_time = feedback.end_time
        # self.diagnostics_traj_pub4.publish(lp)
        self.latest_progress=feedback.completion
        print(feedback)
        self.enable_extruding(feedback.completion > 0.0 and feedback.completion < 1.)

    def joy_cb(self, msg):

        if self.prev_jb0 == 0 and msg.buttons[0] == 1:
            # b0 event
            self.print_rate -= 50
            rospy.loginfo("Current print rate: {}".format(self.print_rate))

        if self.prev_jb3 == 0 and msg.buttons[3] == 1:
            # b3 event
            self.print_rate += 50
            rospy.loginfo("Current print rate: {}".format(self.print_rate))

        self.prev_jb0 = msg.buttons[0]
        self.prev_jb3 = msg.buttons[3]

    def print_donecb(self, _1, _2):
        self.enable_extruding(False)

    def enable_extruding(self, _b):
        _i = Int32()
        _i.data = self.print_rate if _b else _retraction_rate
        self.extruder_pub.publish(_i)
        print("Extruder is {}".format(_b))

# Full trajectory
    def layer2trajectory(self, layer,start_index=0):

        trajectory = TaskTrajectory()
        trajectory.header.frame_id = world_frame
        trajectory.points = list()
        n = len(layer.task_s)
        t = rospy.Duration(0)
        for ii in range(start_index,n):
            pt = TaskPoint()
            pt.ee_pose.position.x = float(layer.task_positions[ii][0]+xo)
            pt.ee_pose.position.y = float(layer.task_positions[ii][1]+yo)
            pt.ee_pose.position.z = float(layer.task_positions[ii][2]+zo)

            pt.ee_pose.orientation.x = float(layer.task_orientations[ii][1])
            pt.ee_pose.orientation.y = float(layer.task_orientations[ii][2])
            pt.ee_pose.orientation.z = float(layer.task_orientations[ii][3])
            pt.ee_pose.orientation.w = float(layer.task_orientations[ii][0])
            #
            pt.base_pose.position.x = float(layer.base_positions[ii][0]+xo)
            pt.base_pose.position.y = float(layer.base_positions[ii][1]+yo)
            pt.base_pose.position.z = 0

            pt.base_pose.orientation.x = float(layer.base_orientations[ii][1])
            pt.base_pose.orientation.y = float(layer.base_orientations[ii][2])
            pt.base_pose.orientation.z = float(layer.base_orientations[ii][3])
            pt.base_pose.orientation.w = float(layer.base_orientations[ii][0])

            pt.tol_elipse.x_tol = min(tol_sat, float(layer.base_elipses[ii][0]))
            pt.tol_elipse.y_tol = min(tol_sat, float(layer.base_elipses[ii][1]))
            pt.tol_elipse.th_tol = min(0.2, float(layer.base_elipses[ii][2]))
            pt.time_from_start = deepcopy(t)
            trajectory.points.append(pt)
            t = t+rospy.Duration(dt/speed_gain)
        return trajectory

    def send_trajectory_diagnostics(self, trajectory):
        # Send Debugs
        _p1 = Marker()
        _p1.type = _p1.LINE_STRIP
        _p1.id = 1
        _p1.action = _p1.ADD
        _p1.header.frame_id = world_frame
        _p1.pose.orientation.w = 1
        _p1.scale.x = 0.001
        _p1.color.g = 0.5
        _p1.color.a = 1.0
        _p1.points = [Point(trajectory.points[ii].ee_pose.position.x, trajectory.points[ii].ee_pose.position.y, trajectory.points[ii].ee_pose.position.z) for ii in range(0, len(trajectory.points))]
        self.diagnostics_traj_pub1.publish(_p1)
        print("Diagnostic1 sent")
        _p2 = PoseArray()
        _p2.header.frame_id = world_frame
        # _p2.poses = [trajectory.points[ii].base_pose for ii in range(0, len(trajectory.points), 20)]
        _p2.poses = [trajectory.points[ii].ee_pose for ii in range(0, len(trajectory.points), 20)]
        self.diagnostics_traj_pub2.publish(_p2)
        print("Diagnostic2 sent")
        self.send_elipses_markers(trajectory)

    def send_elipses_markers(self, _trajectory):
        n = len(_trajectory.points)
        _m = MarkerArray()
        __m = Marker()
        __m.header.frame_id = world_frame
        __m.type = __m.SPHERE
        __m.color.r = 0.1
        __m.color.g = 0.5
        __m.color.b = 0.1
        __m.color.a = 0.015
        __m.action = __m.ADD
        for ii in range(0, n, 20):
            __m.id = 10+ii
            __m.pose.position = _trajectory.points[ii].base_pose.position
            __m.pose.orientation.w = 1
            __m.scale.x = max(_trajectory.points[ii].tol_elipse.x_tol-0.05,0.05)*2
            __m.scale.y = max(_trajectory.points[ii].tol_elipse.y_tol-0.05,0.05)*2
            __m.scale.z = max(_trajectory.points[ii].tol_elipse.th_tol-0.05,0.05)*2
            _m.markers.append(deepcopy(__m))
        self.diagnostics_traj_pub3.publish(_m)


# MATLAB HELPERS from https://stackoverflow.com/questions/7008608/scipy-io-loadmat-nested-structures-i-e-dictionaries
def loadmat(filename):
    '''
    this function should be called instead of direct spio.loadmat
    as it cures the problem of not properly recovering python dictionaries
    from mat files. It calls the function check keys to cure all entries
    which are still mat-objects
    '''
    data = spio.loadmat(filename, struct_as_record=False, squeeze_me=True)
    return _check_keys(data)


def _check_keys(dict):
    '''
    checks if entries in dictionary are mat-objects. If yes
    todict is called to change them to nested dictionaries
    '''
    for key in dict:
        if isinstance(dict[key], spio.matlab.mio5_params.mat_struct):
            dict[key] = _todict(dict[key])
    return dict


def _todict(matobj):
    '''
    A recursive function which constructs from matobjects nested dictionaries
    '''
    dict = {}
    for strg in matobj._fieldnames:
        elem = matobj.__dict__[strg]
        if isinstance(elem, spio.matlab.mio5_params.mat_struct):
            dict[strg] = _todict(elem)
        else:
            dict[strg] = elem
    return dict


if __name__ == "__main__":
    rospy.init_node('tasker')
    pt = Tasker()
    rospy.spin()
    pt.enable_extruding(False)
