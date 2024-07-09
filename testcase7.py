#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import sys
import select

cancel_flag = False

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def check_for_cancel():
    global cancel_flag
    if select.select([sys.stdin], [], [], 0.1)[0]:
        if sys.stdin.readline().strip().lower() == 'cancel':
            cancel_flag = True

def main():
    global cancel_flag

    # Initializing ROS2 communications and Simple Commander API
    rclpy.init()
    nav = BasicNavigator()

    # Setting initial pose (home position)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # Wait for Nav2 initialization
    nav.waitUntilNav2Active()

    # Get table numbers from console
    table_numbers = input("Enter table numbers (comma-separated): ")
    table_numbers = [int(num.strip()) for num in table_numbers.split(',')]

    # Define poses for kitchen and tables
    kitchen_pose = create_pose_stamped(nav, -3.0, -0.8, 0.0)
    table_poses = {
        1: create_pose_stamped(nav, 0.5, 2.0, 0.0),
        2: create_pose_stamped(nav, -3.1, 3.2, 0.0),
        3: create_pose_stamped(nav, -5.9, 1.6, 0.0)
    }

    # Move to kitchen
    nav.goToPose(kitchen_pose)
    while not nav.isTaskComplete():
        check_for_cancel()
        if cancel_flag:
            nav.goToPose(initial_pose)
            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
            return
        feedback = nav.getFeedback()

    # Move to each table
    for table_number in table_numbers:
        if table_number in table_poses:
            nav.goToPose(table_poses[table_number])
            while not nav.isTaskComplete():
                check_for_cancel()
                if cancel_flag:
                    print(f'Order for table {table_number} canceled. Moving to the next table.')
                    cancel_flag = False  # Reset the cancel flag
                    break
                feedback = nav.getFeedback()
            else:
                continue  # Continue to the next table if no cancellation occurred

    # Return to kitchen after final delivery
    nav.goToPose(kitchen_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()

    # Return to home position
    nav.goToPose(initial_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()

    # Get the result
    print(nav.getResult())

    # Shutdown ROS2 communications
    rclpy.shutdown()

if __name__ == '__main__':
    main()
