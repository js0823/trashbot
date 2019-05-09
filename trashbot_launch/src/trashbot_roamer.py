#!/usr/bin/env python

import rospy
import actionlib
import tf
import geometry_msgs.msg
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

def yolo_callback(yolo_boxes):
    if not yolo_boxes:
        return 0
    else:
        s = str(yolo_boxes.yolo_boxes[0])
        stringArr = re.findall(r"[-+]?\d*\.\d+|\d+", s)
        # stringArr = ['0.9996', '300', '326', '324', '406']
        # bounding box pixel area size = (xmax - xmin) * (ymax - ymin)
        boxAreaSize = (int(stringArr[2]) - int(stringArr[1])) * \
                        (int(stringArr[4]) - int(stringArr[3]))
        prob = float(stringArr[0])
        return [prob, boxAreaSize]

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
        rospy.init_node('trashbot_roamer_py')

        # subscribe to yolo publisher
        input_topic = rospy.get_param('~input_topic', "/yolo/result")

        home_location = [5.65, 13.8, 0.0]

        num_locations = 2
        locations = [[21.8,13.9,0.0], [5.8,13.9,0.0]]

        location_names = ["Collab Room Left", "Kitchen Left"]
        x1 = y1 = x2 = y2 = x3 = y3 = x4 = y4 = 0
        start_index = 1
        goal_index = start_index

        # Trash location
        trash_location = []

        # Sound play
        soundHandle = SoundClient()
        voice = 'voice_kal_diphone'
        volume = 1.0

        while not rospy.is_shutdown():
            # subscribe to yolo publisher
            yolo_sub = rospy.Subscriber(input_topic, YoloBoxes, yolo_callback, queue_size=1)
            print(yolo_sub[0])
            print(yolo_sub[1])
            '''
            # If trash is found, add the location
            if yolo_sub > 0 and not trash_location:
                trash_location.append(yolo_sub)

            if not trash_location: # No trash found
                print("Target Location : {}".format(location_names[goal_index]))
                # Move to location
                result = move_turtlebot(locations[goal_index][0], locations[goal_index][1],
                                        locations[goal_index][2])
                # Rotate around
                for p in range(3):
                    result = move_turtlebot(locations[goal_index][0], locations[goal_index][1], 
                                            locations[goal_index][2] + (p + 1) * PI / 2)
                if result:
                    rospy.loginfo("Goal execution done.")
                rospy.sleep(10)
                goal_index += 1
                if goal_index >= num_locations:
                    goal_index = 0
            else: # trash found. Go here first.
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
                rospy.sleep(10)
                # Remove trash location
                trash_location[:] = []
            '''
            
    except rospy.ROSInternalException:
        rospy.loginfo("Roamer finished.")