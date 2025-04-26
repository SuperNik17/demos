#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def send_goal(position, orientation):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'robot/odom'
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position of the waypoint
    goal.target_pose.pose.position = Point(*position)

    # Set the orientation of the waypoint
    goal.target_pose.pose.orientation = Quaternion(*orientation)

    # Send the goal to move_base
    client.send_goal(goal, done_cb=goal_completion_callback)

def goal_completion_callback(status, result):
    if status == 3:  # Goal reached successfully
        print("Goal reached!")

        # Continue to the next goal
        send_next_goal()

def send_next_goal():
    global waypoint_index

    # Check if there are more waypoints
    if waypoint_index < len(waypoints):
        position, orientation = waypoints[waypoint_index]
        send_goal(position, orientation)
        waypoint_index += 1
    else:
        waypoint_index = 0
        send_next_goal()
        print("All waypoints reached. \n Repeat...")

def main():
    # Initialize ROS node
    rospy.init_node('send_waypoints')


    global waypoints
    waypoints = []
    f = open('/home/agilex/catkin_ws/src/unina_nav_pkg/scripts/collected_waypoints_cart.txt')
    lines = f.readlines()
    for line in lines: 
        coordinate = line.split()
        points = [float(coordinate[0]), float(coordinate[1]), float(coordinate[2])]
        quaternions = [float(coordinate[3]), float(coordinate[4]), float(coordinate[5]), float(coordinate[6])]
        waypoints.append(  (points, [0, 0, 0, 1]) )

    global waypoint_index
    waypoint_index = 0

    # Start sending goals
    send_next_goal()

    rospy.spin()

if __name__ == '__main__':
    main()
