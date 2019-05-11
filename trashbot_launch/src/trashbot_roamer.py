#!/usr/bin/env python

import rospy
import actionlib
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf_conversions
import yolo_detector
import re
from yolo_ros_msgs.msg import YoloBox
from yolo_ros_msgs.msg import YoloBoxes

# sound_play
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

PI = 3.14159265359
robot_pose = None # robot's current position
trash_location = [] # trash location

def yolo_callback(yolo_boxes):
    global trash_location
    if yolo_boxes.yolo_boxes != []:
        s = str(yolo_boxes.yolo_boxes[0])
        stringArr = re.findall(r"[-+]?\d*\.\d+|\d+", s)
        # stringArr = ['0.9996', '300', '326', '324', '406']
        # bounding box pixel area size = (xmax - xmin) * (ymax - ymin)
        boxAreaSize = (int(stringArr[2]) - int(stringArr[1])) * \
                        (int(stringArr[4]) - int(stringArr[3]))
        prob = float(stringArr[0])
        if prob > 0.6 and boxAreaSize > 1000:
            if len(trash_location) == 0:
                trash_location.append(robot_pose)
                print(trash_location)

def current_pos_callback(pose):
    global robot_pose
    x = pose.pose.pose.position.x
    y = pose.pose.pose.position.y
    z = 0.0
    robot_pose = [x, y, z]

def move_turtlebot(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    print("Target Position: {}, {}".format(x, y))

    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "/map"

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                    *tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        global trash_location
        global robot_pose
        rospy.init_node('trashbot_roamer_py')

        # subscribe to yolo publisher
        input_topic = rospy.get_param('~input_topic', "/yolo/result")
        # subscribe to yolo publisher
        yolo_sub = rospy.Subscriber(input_topic, YoloBoxes, yolo_callback, queue_size=1)
        # subscribe to turtlebot amcl for base_link position
        amcl_sub = rospy.Subscriber('/amcl_pose', 
                    PoseWithCovarianceStamped, current_pos_callback, queue_size=1)

        num_locations = 5
        locations = [[21.8,13.9,0.0], [17.8,13.9,0.0], [13.8,13.9,0.0], [9.8,13.9,0.0], [5.8,13.9,0.0]]

        location_names = ["Collab Room Left", "Close to Collab Room Left", "Midpoint", 
                            "Close to Kitchen Left", "Kitchen Left"]
        start_index = 1
        goal_index = start_index

        # Sound play
        soundHandle = SoundClient()
        voice = 'voice_kal_diphone'
        volume = 1.0

        while not rospy.is_shutdown():
            if len(trash_location) == 0: # No trash position, so roam around
                print("Target Location : {}".format(location_names[goal_index]))
                # Move to location
                result = move_turtlebot(locations[goal_index][0], locations[goal_index][1],
                                        locations[goal_index][2])
                # Rotate around
                if goal_index == 0 or goal_index == num_locations - 1:
                    for p in range(3):
                        result = move_turtlebot(locations[goal_index][0], locations[goal_index][1], 
                                                locations[goal_index][2] + (p + 1) * PI / 2)
                if result:
                    rospy.loginfo("Goal execution done.")
                rospy.sleep(2)
                goal_index += 1
                if goal_index >= num_locations:
                    goal_index = 0
            else: # trash found while navigating.
                print("Going to trash.")
                # Move to location
                result = move_turtlebot(trash_location[0][0], trash_location[0][1], 
                                        trash_location[0][2])
                # Rotate around
                for p in range(3):
                    result = move_turtlebot(trash_location[0][0], trash_location[0][1], 
                                            trash_location[0][2] + (p + 1) * PI / 2)
                if result:
                    rospy.loginfo("Going to trash: done.")
                # Make a sound
                soundHandle.say('Trash Detected. Please pick it up.', 
                                            voice, volume)
                rospy.sleep(5)
                # Remove trash location
                trash_location[:] = []
            
    except rospy.ROSInternalException:
        rospy.loginfo("Roamer finished.")