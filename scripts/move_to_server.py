#!/usr/bin/env python3

import rospy
import rospkg
import yaml
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from mirte_navigation.srv import MoveTo, MoveToResponse  # Import the custom service

class MoveToLocation:
    def __init__(self):
        rospy.init_node('move_to_location_node')
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('mirte_workshop')
        self.locations_dir = os.path.join(self.package_path, "maps")
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base to come online")
        self.move_base_client.wait_for_server()
        self.service = rospy.Service('move_to', MoveTo, self.handle_move_to)
        rospy.loginfo("rosservice '/move_to' now available")

    def load_locations(self, map_file_name):
        locations_file_path = os.path.join(self.locations_dir, f'{map_file_name}.yaml')
        if not os.path.exists(locations_file_path):
            rospy.logwarn(f"No locations file found for {map_file_name}")
            return {}
        
        with open(locations_file_path, 'r') as file:
            locations_data = yaml.safe_load(file)
        
        if not locations_data:
            rospy.logwarn(f"No locations data in file {locations_file_path}")
            return {}

        return locations_data

    def handle_move_to(self, req):
        location_label = req.location
        locations_data = self.load_locations('stored_poses')

        if location_label not in locations_data:
            message = "location unknown"
            self.status_pub.publish(message)
            return MoveToResponse(success=False, message=message)

        location = locations_data[location_label]
        position = location['position']
        orientation = location['orientation']

        # Cancel all current goals
        self.move_base_client.cancel_all_goals()

        # Create a new goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position['x']
        goal.target_pose.pose.position.y = position['y']
        goal.target_pose.pose.position.z = position['z']
        goal.target_pose.pose.orientation.x = orientation['x']
        goal.target_pose.pose.orientation.y = orientation['y']
        goal.target_pose.pose.orientation.z = orientation['z']
        goal.target_pose.pose.orientation.w = orientation['w']

        message = f"moving to {location_label}"
        self.status_pub.publish(message)
        self.move_base_client.send_goal(goal, done_cb=self.move_base_done_cb)
        
        return MoveToResponse(success=True, message=message)

    def move_base_done_cb(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            message = "Move base action succeeded."
        else:
            message = "Move base action failed."
        
        self.status_pub.publish(message)

if __name__ == '__main__':
    try:
        MoveToLocation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
