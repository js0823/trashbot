#!/usr/bin/env python

import rospy
import actionlib
import tf
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf_conversions
import time
import yolo_detector

PI = 3.14159265359

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
        home_location = [5.65, 13.8, 0.0]

        num_locations = 2
        locations = [[21.8,13.9,0.0], [5.8,13.9,0.0]]

        location_names = ["Collab Room Left", "Kitchen Left"]
        x1 = y1 = x2 = y2 = x3 = y3 = x4 = y4 = 0
        start_index = 1
        goal_index = start_index

        # Trash locations
        num_trash_location = 1
        trash_location = []

        while not rospy.is_shutdown():
            if not trash_location: # No trash found
                print("Target Location : {}".format(location_names[goal_index]))
                # Move to location
                result = move_turtlebot(locations[goal_index][0],
                                        locations[goal_index][1],locations[goal_index][2])
                # Rotate around
                for p in range(3):
                    result = move_turtlebot(locations[goal_index][0], 
                                            locations[goal_index][1], locations[goal_index][2]+(p+1)*PI/2)
                if result:
                    rospy.loginfo("Goal execution done.")
                rospy.sleep(10)
                goal_index += 1
                if goal_index >= num_locations:
                    goal_index = 0
            else:
                print("Going to trash.")
                # Move to location
                result = move_turtlebot(trash_location[goal_index][0],
                                        trash_location[goal_index][1], trash_location[goal_index][2])
                # Rotate around
                for p in range(3):
                    result = move_turtlebot(trash_location[goal_index][0], trash_location[goal_index][1], 
                                            trash_location[goal_index][2]+(p+1)*PI/2)
                if result:
                    rospy.loginfo("Going to trash: done.")
                rospy.sleep(10)

    except rospy.ROSInternalException:
        rospy.loginfo("Roamer finished.")