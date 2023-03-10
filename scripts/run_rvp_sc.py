#!/usr/bin/env python3
import sys
import rospy

import roslaunch
import time
import os
from roi_viewpoint_planner_msgs.srv import StartEvaluator, StartEvaluatorRequest, StartEvaluatorResponse
import datetime
import subprocess


parent_dir = "/home/rohit/workspace/ros1/rvp-ros-workspaces/ros_rvp/devel_release/results"

sc_dict = {
    "superellipsoids": 0,
    "shape_registration": 1
}

roi_sampling_dict = {
    "roi_contours": 3 ,
    "roi_adjacent": 4,
    "predicted_rois": 6,
    "predicted_roi_centres": 7
}

scenario_dict = {
    "scenario_1": ['world_name:=world14_hc', 'base:=static'],
    "scenario_2": ['world_name:=world19', 'base:=retractable'],
    "scenario_3": ['world_name:=world22', 'base:=retractable']
}

class LaunchFile:
    def __init__(self, package, file, args=[]):
        self.file = [(roslaunch.rlutil.resolve_launch_arguments([package, file])[0], args)]
        self.is_launched = False

    def launch(self, uuid):
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, self.file)
        self.parent.start()
        self.is_launched = True
        return self.parent

    def spin(self):
        self.parent.spin_once()

    def shutdown(self):
        print('Shutting down')
        self.parent.shutdown()
        print('Shutdown complete')



class ComponentLaunch:
    def __init__(self, roi_sampling_method="predicted_rois", shape_completion_method="superellipsoids", scenario="scenario_1"):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # current_date = datetime.datetime.now()
        # date_str = current_date.strftime("%Y%m%d_%H%M")

        # hr        = datetime.datetime.now().hour   # the current hour
        # min      = datetime.datetime.now().minute # the current minute
       


        # new_dir = date_str + "_" + roi_sampling_method + "_" + shape_completion_method + "_" + scenario

        # # Path
        # path = os.path.join(parent_dir, new_dir)

        # os.mkdir(path)
        # print("Directory '% s' created" % new_dir)

        # full_dir = parent_dir + "/" + new_dir
        # print(full_dir)
        # print("Current working directory: {0}".format(os.getcwd()))

        # # Change the current working directory
        # os.chdir(path)

        # # bash_command = "cd "+ full_dir
        # # process = subprocess.Popen(["echo", full_dir], stdout=subprocess.PIPE)
        # # output, error = process.communicate()


        # # Print the current working directory
        # print("Current working directory: {0}".format(os.getcwd()))

        self.roi_sampling_method = roi_sampling_method
        self.shape_completion_method = shape_completion_method
        self.scenario = scenario

        self.world_launch = LaunchFile('ur_with_cam_gazebo', 'ur_with_cam.launch', scenario_dict[self.scenario])


        self.shape_completion_launch = LaunchFile('capsicum_superellipsoid_detector', 'start_sim.launch')
        self.rqt_launch = LaunchFile('roi_viewpoint_planner', 'launch_rqt.launch')


    def start(self):
        self.world_launch.launch(self.uuid)
        time.sleep(5)
        rospy.init_node("rvp_sc")
        rospy.set_param("scenario", self.scenario)
        self.shape_completion_launch.launch(self.uuid)
        time.sleep(5)
        self.rqt_launch.launch(self.uuid)

    def shutdown(self):
        self.shape_completion_launch.shutdown()
        time.sleep(5)
        self.world_launch.shutdown()
        time.sleep(5)
        self.rqt_launch.shutdown()
        time.sleep(5)

class EvaluatorConfig:
    def __init__(self, _num_evals=10, _episode_end_param= 2, _episode_duration=300.0):
        rospy.wait_for_service('/roi_viewpoint_planner/start_evaluator')
        resp = False
        try:
            start_eval = rospy.ServiceProxy('/roi_viewpoint_planner/start_evaluator', StartEvaluator)
            resp = start_eval(num_evals =_num_evals, episode_end_param =_episode_end_param, episode_duration = _episode_duration)
            
        except rospy.ServiceException as e:
            raise Exception("Evaluator Service call failed: %s"%e)
        
        if resp.success == False:
            raise Exception("Evaluator Service call returned false: %s"%e) 

if __name__ == "__main__":

    rvp_sc = ComponentLaunch( roi_sampling_method="predicted_rois", shape_completion_method="superellipsoids", scenario="scenario_3")
    try:
        rvp_sc.start()
        #eval_config= EvaluatorConfig(_num_evals=10, _episode_end_param= 2, _episode_duration=300.0)
        rospy.spin()
    except:
        rospy.logwarn("Exception!!! Shutting all nodes down")
        rvp_sc.shutdown()
    finally:
        rospy.logwarn("Shutting all nodes down")
        rvp_sc.shutdown()
