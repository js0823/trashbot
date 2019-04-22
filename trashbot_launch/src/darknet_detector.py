#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
import sys

from ctypes import *
import math
import random
import cv2
import colorsys
import numpy as np
import os
import shutil
import copy