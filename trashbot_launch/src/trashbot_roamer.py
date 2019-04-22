#!/usr/bin/env python

import rospy
import actionlib
import tf
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf_conversions

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
    #goal.target_pose.pose.orientation = tf.createQuaternionMsgFromYaw(yaw)
    #quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    #goal.target_pose.pose.orientation.x = quat[0]
    #goal.target_pose.pose.orientation.y = quat[1]
    #goal.target_pose.pose.orientation.z = quat[2]
    #goal.target_pose.pose.orientation.w = quat[3]

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

        while not rospy.is_shutdown():
            print("Target Location : {}".format(location_names[goal_index]))
            for p in range(3):
                result = move_turtlebot(locations[goal_index][0], locations[goal_index][1], locations[goal_index][2] + (p + 1) * PI / 2)
            if result:
                rospy.loginfo("Goal execution done.")
            goal_index += 1
            if goal_index >= num_locations:
                goal_index = 0
    except rospy.ROSInternalException:
        rospy.loginfo("Roamer finished.")