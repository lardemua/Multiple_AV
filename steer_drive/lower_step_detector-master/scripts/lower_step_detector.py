#!/usr/bin/env python
# coding: utf-8

"""
fix original laser scan data to detect down step
"""
__author__  = "MasaruMorita <p595201m@mail.kyutech.jp>"
__version__ = "1.01"
__date__    = "18 Oct 2015"

import rospy
import copy
import math
import sys
import numpy as np
from sensor_msgs.msg import LaserScan

NAME_NODE = 'lower_step_detector'

class LaserScanEx():
    threshold_intensity = 0
    angle_laser_rad = 0
    angle_laser_deg = 0
    angle_laser_to_calc_intensity = 0
    virtual_intensity_to_ground = 0
    is_beyond_threshold_before = False
    is_beyond_threshold_after = False

    def __init__(self):
        self.threshold = 0
        self.angle_laser_rad = 0
        self.angle_laser_deg = 0
        self.angle_laser_to_calc_intensity = 0
        self.is_beyond_threshold_before = False
        self.is_beyond_threshold_after = False
        self.intensity_to_ground = 0

class ChunkAngle():
    index_start_laser_range = 0
    index_end_laser_range = 0
    count_elem = 0
    count_beyond_thresh = 0
    thresh_ratio_detect = 0
    ratio_detect = 0
    is_beyond_threshold = False

    def __init__(self, i_count_elem, i_index_start, i_thresh_ratio_detect):
        self.count_elem = i_count_elem
        self.index_start_laser_range = i_index_start
        self.index_end_laser_range = self.index_start_laser_range + self.count_elem
        self.thresh_ratio_detect = i_thresh_ratio_detect

    def judge_beyond_thresh(self, i_laser_scan_exs):
        self.count_beyond_thresh = 0
        for i in range(int(self.index_start_laser_range), int(self.index_end_laser_range)):
            if i_laser_scan_exs[i].is_beyond_threshold_before == True:
                self.count_beyond_thresh += 1

        self.ratio_detect = self.count_beyond_thresh / self.count_elem
        if self.ratio_detect > self.thresh_ratio_detect:
            self.is_beyond_threshold = True
        else:
            self.is_beyond_threshold = False

    def update_intensity(self, i_tmp_fix_data, i_laser_scan_exs):
        if self.is_beyond_threshold == True:
            for i in range(int(self.index_start_laser_range), int(self.index_end_laser_range)):
                i_tmp_fix_data[i] = copy.deepcopy(i_laser_scan_exs[i].virtual_intensity_to_ground)
                #i_tmp_fix_data[i] = 1.0#i_laser_scan_exs[i].virtual_intensity_to_ground

class LowerStepDetector():
    ## public constants
    # topic names
    NAME_TOPIC_SUFFIX_TO_PUBLISH = '_fix'
    NAME_TOPIC_LASER_ORI = 'base_scan1'
    NAME_TOPIC_LASER_FIX = NAME_TOPIC_LASER_ORI + NAME_TOPIC_SUFFIX_TO_PUBLISH
    # parameter names
    NAME_PARAM_LASER_INTENSITY = NAME_NODE + '/laser_intensity_max'
    NAME_PARAM_VIRTUAL_LASER_INTENSITY = NAME_NODE + '/virtual_laser_intensity'
    NAME_PARAM_LASER_SCAN_RANGE_DEG = NAME_NODE + '/laser_scan_range_deg'
    NAME_PARAM_DETECT_STEP_ANGLE_MIN_DEG = NAME_NODE + '/detect_step_angle_min_deg'
    NAME_PARAM_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP = NAME_NODE + '/margin_between_plane_and_down_step'
    NAME_PARAM_CHUNK_ANGLE_FOR_NOISE_DEG = NAME_NODE + '/chunk_angle_for_noise_deg'
    NAME_PARAM_RATIO_DETECT_IN_CHUNK = NAME_NODE + '/ratio_detect_in_chunk'

    # default parameter values
    DEFAULT_LASER_INTENSITY_MAX = 1.5
    DEFAULT_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP = 1.0
    DEFAULT_VIRTUAL_LASER_INTENSITY = 1.0
    DEFAULT_LASER_SCAN_RANGE_DEG = 180.0
    DEFAULT_DETECT_STEP_ANGLE_MIN_DEG = 10.0
    DEFAULT_CHUNK_ANGLE_FOR_NOISE_DEG = 5.0
    DEFAULT_RATIO_DETECT_IN_CHUNK = 0.8

    ## private member variables
    # original laser scan subscriber
    _laser_ori_sub = 0
    # fixed laser scan publisher
    _laser_fix_pub = 0
    _laser_intensity_max = 0
    _virtual_laser_intensity = 0
    _detect_index_min = 0
    _detect_index_max = 0
    #
    _laser_scan_range_deg = 0
    _detect_step_angle_min_deg = 0
    _detect_step_angle_max_deg = 0
    _detect_angle_center_deg = 0
    _margin_between_plane_and_down_step = 0
    _chunk_angle_for_noise_deg = 0
    _ratio_detect_in_chunk = 0
    #
    _is_init = True
    _laser_scan_exs = []

    _count_chunk_angle = 0
    _chunks_to_detect_noise = []

    ## constructor
    def __init__(self):
        rospy.init_node(NAME_NODE, anonymous=False)
        self.load_topic_name_to_sub()
        self.load_rosparam()
        self._laser_ori_sub = rospy.Subscriber(self.NAME_TOPIC_LASER_ORI, LaserScan, self.on_subscribe_laser_scan)
        self._laser_fix_pub = rospy.Publisher(self.NAME_TOPIC_LASER_FIX, LaserScan, queue_size = 1)
        # start callback
        rospy.spin()

    ## methods
    def load_topic_name_to_sub(self):
        argvs = sys.argv
        count_arg = len(argvs)
        if count_arg > 1:
            self.NAME_TOPIC_LASER_ORI = argvs[1]
            self.NAME_TOPIC_LASER_FIX = self.NAME_TOPIC_LASER_ORI + self.NAME_TOPIC_SUFFIX_TO_PUBLISH

    def load_rosparam(self):
        self._laser_intensity_max = rospy.get_param(self.NAME_PARAM_LASER_INTENSITY, self.DEFAULT_LASER_INTENSITY_MAX)
        self._margin_between_plane_and_down_step = rospy.get_param(self.NAME_PARAM_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP, self.DEFAULT_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP)
        self._virtual_laser_intensity = rospy.get_param(self.NAME_PARAM_VIRTUAL_LASER_INTENSITY, self.DEFAULT_VIRTUAL_LASER_INTENSITY)
        self._laser_scan_range_deg = rospy.get_param(self.NAME_PARAM_LASER_SCAN_RANGE_DEG, self.DEFAULT_LASER_SCAN_RANGE_DEG)
        self._detect_step_angle_min_deg = rospy.get_param(self.NAME_PARAM_DETECT_STEP_ANGLE_MIN_DEG, self.DEFAULT_DETECT_STEP_ANGLE_MIN_DEG)
        self._detect_step_angle_max_deg = self._laser_scan_range_deg - self._detect_step_angle_min_deg
        self._detect_angle_center_deg = self._laser_scan_range_deg / 2.0
        self._chunk_angle_for_noise_deg = rospy.get_param(self.NAME_PARAM_CHUNK_ANGLE_FOR_NOISE_DEG, self.DEFAULT_CHUNK_ANGLE_FOR_NOISE_DEG)
        self._ratio_detect_in_chunk = rospy.get_param(self.NAME_PARAM_RATIO_DETECT_IN_CHUNK, self.DEFAULT_RATIO_DETECT_IN_CHUNK)

    def calculate_threshold_intensity(self, i_laser_sensor_msg_ori):
         # calculate parameters
         angle_increment = i_laser_sensor_msg_ori.angle_increment
         self._detect_index_min = math.radians(self._detect_step_angle_min_deg) / angle_increment
         self._detect_index_max = math.radians(self._detect_step_angle_max_deg) / angle_increment
         detect_index_mid = math.radians(self._detect_angle_center_deg) / angle_increment

         for i in range(len(i_laser_sensor_msg_ori.ranges)):
             laser_scan_ex = LaserScanEx()

             angle_curr_rad = i * angle_increment
             angle_curr_deg = math.degrees(angle_curr_rad)

             if i < detect_index_mid:
                 theta = angle_curr_rad
             else:
                 theta = math.pi - angle_curr_rad

             # to avoid zero division
             if theta != 0:
                 virtual_intensity_to_ground = self._virtual_laser_intensity / math.sin(theta)
                 laser_intensity_thresh = (self._laser_intensity_max  + self._margin_between_plane_and_down_step) / math.sin(theta)
             else:
                 virtual_intensity_to_ground = self._virtual_laser_intensity
                 laser_intensity_thresh = float("inf")

             laser_scan_ex.virtual_intensity_to_ground = virtual_intensity_to_ground
             laser_scan_ex.angle_laser_rad = angle_curr_rad
             laser_scan_ex.angle_laser_deg = angle_curr_deg
             laser_scan_ex.angle_laser_to_calc_intensity = theta

             # beyond the extend of step scan
             if i < self._detect_index_min or i > self._detect_index_max:
                 laser_scan_ex.threshold_intensity = float("inf")
             else:
                 laser_scan_ex.threshold_intensity = laser_intensity_thresh
             self._laser_scan_exs.append(laser_scan_ex)

    def create_angle_chunk(self, i_laser_sensor_msg_ori):
        self.calculate_threshold_intensity(i_laser_sensor_msg_ori)

        if (self._chunk_angle_for_noise_deg != 0):
            count_chunk = round(self._laser_scan_range_deg / self._chunk_angle_for_noise_deg)
        else:
            count_chunk = 6

        count_elem_chunk = len(i_laser_sensor_msg_ori.ranges) / count_chunk

        for j in range(int(count_chunk)):
            index_start = count_elem_chunk * j
            chunk_angle = ChunkAngle(count_elem_chunk, index_start, self._ratio_detect_in_chunk)
            self._chunks_to_detect_noise.append(chunk_angle)

    # first intensity judge. before analyzing intensity
    def judge_intensity(self, i_laser_sensor_msg_ori):
        for i in range(len(i_laser_sensor_msg_ori.ranges)):
            # skip when a range cannot detect down step
            if i < self._detect_index_min or i > self._detect_index_max:
                continue

            # overwrite only when range can detect down step
            if i_laser_sensor_msg_ori.ranges[i] > self._laser_scan_exs[i].threshold_intensity:
                #print 'detected lower step at %f[degree]!' % self._laser_scan_exs[i].angle_laser_deg
                self._laser_scan_exs[i].is_beyond_threshold_before = True
            else:
                self._laser_scan_exs[i].is_beyond_threshold_before = False

    # second intensity judge. analyzing intensity for noise detect
    def analyze_intensity_noise_detect(self, i_tmp_fix_data):
        for j in range(len(self._chunks_to_detect_noise)):
            self._chunks_to_detect_noise[j].judge_beyond_thresh(self._laser_scan_exs)
            self._chunks_to_detect_noise[j].update_intensity(i_tmp_fix_data, self._laser_scan_exs)

    def on_subscribe_laser_scan(self, i_laser_sensor_msg_ori):
        laser_sensor_msg_fix = copy.deepcopy(i_laser_sensor_msg_ori)
        # temporary buffer for publish. This is because tuple(type of LaserScan.ranges) type can't be overwritten.
        tmp_fix_data = len(i_laser_sensor_msg_ori.ranges)*[0]

        # calculate threshold only when it's the first time
        if self._is_init == True:
            # reset initial flag
            self._is_init = False
            self.create_angle_chunk(i_laser_sensor_msg_ori)

        # judge laser scan data
        for i in range(len(i_laser_sensor_msg_ori.ranges)):
            # copy original data
            tmp_fix_data[i] = copy.deepcopy(i_laser_sensor_msg_ori.ranges[i])

        # judge laser scan data before noise detect
        self.judge_intensity(i_laser_sensor_msg_ori)

        # analyze laser scan data to consider noise
        self.analyze_intensity_noise_detect(tmp_fix_data)

        laser_sensor_msg_fix.ranges = tuple(tmp_fix_data)

        # publish fixed laser scan topic
        self._laser_fix_pub.publish(laser_sensor_msg_fix)
        #print 'succeed publish'

if __name__ == '__main__':
    try:
        print 'start program'
        LowerStepDetector()

    except:
        rospy.loginfo("lower_step_detector finished.")
