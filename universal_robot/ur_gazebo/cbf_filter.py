#!/usr/bin/env python3
import rospy
import geometry_msgs.msg as geometry_msgs
import numpy as np
import math
from geometry_msgs.msg import Pose, PoseArray

avoidance_point = Pose()
avoidance_point.position.x = 0
avoidance_point.position.y = 0.45
avoidance_point.position.z = 0.2
safety_distance = 0.2

pub = rospy.Publisher('/adjusted_task_waypoints', PoseArray, queue_size=10)

def cbf_filter(data):
    print("subscribe")
    adjusted_path_points = PoseArray()
    #print(data.poses)
    for intermediated_cart_pose in data.poses:
        diff_x = intermediated_cart_pose.position.x - avoidance_point.position.x
        diff_y = intermediated_cart_pose.position.y - avoidance_point.position.y
        diff_z = intermediated_cart_pose.position.z - avoidance_point.position.z
        
        distance_obs = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        
        if distance_obs < safety_distance:
            direction_vector = [diff_x, diff_y, diff_z]
            norm = math.sqrt(sum([component**2 for component in direction_vector]))
            unit_vector = [component / norm for component in direction_vector]
                
            new_pose = Pose()
            new_pose.position.x = avoidance_point.position.x + safety_distance * unit_vector[0]
            new_pose.position.y = avoidance_point.position.y + safety_distance * unit_vector[1]
            new_pose.position.z = avoidance_point.position.z + safety_distance * unit_vector[2]
            new_pose.orientation = intermediated_cart_pose.orientation
            adjusted_path_points.poses.append(new_pose)
        else:
            adjusted_path_points.poses.append(intermediated_cart_pose)
    
    pub.publish(adjusted_path_points)
    print(adjusted_path_points)
    print("publish")

def main():
    rospy.init_node('path_adjustment_node', anonymous=True)
    rospy.Subscriber('/mat_task_waypoints', PoseArray, cbf_filter)
    rospy.spin()

if __name__ == '__main__':
    main()

