#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from ctypes import *
import math
import random
import cv2
import colorsys
import numpy as np
import os
import shutil
import copy
# import argparse

from darknetv2 import *
from yolo_ros_msgs.msg import YoloBox
from yolo_ros_msgs.msg import YoloBoxes

# sound_play
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class YoloDetectorNode:
    def __init__(self):
        rospy.init_node('YoloDetectorNode')
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('trashbot_launch')
        net_file = file_path + rospy.get_param('~net_file', "/../darknet_network_config/cfg/yolov3-tiny.cfg")
        weights_file = file_path + rospy.get_param('~weights_file', "/../darknet_network_config/weights/yolov3-tiny_50000.weights")
        meta_file = file_path + rospy.get_param('~meta_file', "/../darknet_network_config/cfg/trashnet.data")
        names_file = file_path + rospy.get_param('~names_file', "/../darknet_network_config/cfg/trashnet.names")
        input_image_topic = rospy.get_param('~input_image_topic', "/camera/rgb/image_raw")
        output_image_topic = rospy.get_param('~output_image_topic', "/camera/rgb/yolo_image_output")
        output_topic = rospy.get_param('~output_topic', "/yolo/recognition_result")

        if os.path.isdir('data'):
            shutil.rmtree('data')

        os.mkdir("data")
        shutil.copy(names_file, "data")
        self.net = load_net(net_file.encode('utf-8'), weights_file.encode('utf-8'), 0)
        self.meta = load_meta(meta_file.encode('utf-8'))
        shutil.rmtree("data")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.result_image_pub = rospy.Publisher(output_image_topic, Image, queue_size=1)
        self.result_pub = rospy.Publisher(output_topic, YoloBoxes, queue_size=1)

        self.color = (0,255,0)

        # sound_play
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0

        rospy.loginfo("Initialized yolo_detector_node")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detect = self.darknet_detection(cv_image)
            if detect == True:
                self.soundhandle.say('Trash Detected. Please pick it up.', self.voice, self.volume)
        except CvBridgeError as e:
            print(e)

    def darknet_detection(self, cv_image):

        # cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)

        yolo_boxes = YoloBoxes()
        yolobox_list = []

        cv_image_ = copy.deepcopy(cv_image)

        r = detect(self.net, self.meta, cv_image)
        for i in r:
            x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
            xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
            probability = round(i[1], 4)
            label = i[0].decode()

            yolobox = YoloBox()
            yolobox.probability = probability
            yolobox.label = label
            yolobox.xmin = xmin
            yolobox.ymin = ymin
            yolobox.xmax = xmax
            yolobox.ymax = ymax

            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            cv2.rectangle(cv_image_, pt1, pt2, self.color, 2)
            cv2.putText(cv_image_, label + " [" + str(probability*100) + "%]",
                        (pt1[0]+2, pt1[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color, 3)

            yolobox_list.append(yolobox)

        yolo_boxes.yolo_boxes = yolobox_list
        self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_, "bgr8"))
        self.result_pub.publish(yolo_boxes)

        if r == []:
            return False
        else:
            return True

if __name__ == "__main__":
    try:
        YoloDetectorNode()
        rospy.spin()
    except KeyboardInterrupt:
         print("Shutting down")