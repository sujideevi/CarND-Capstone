#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS_SAMPLES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200]
MAX_DECEL = 0.5 # Max deceleration
STOPPING_WPS_BEFORE = 4 # Number of waypoints to stop before a traffic light line

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Creating publisher object for Final waypoint 
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None        
        self.waypoints_tree = None
        self.stop_line_wp_idx = -1
        
        self.loop()
        
    def loop(self):
        rate = rospy.Rate(50)   # FREQUENCY
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoints_tree:
                final_waypoints = self.get_final_waypoints()
                self.publish_waypoints(final_waypoints)
            rate.sleep()
            
    def get_final_waypoints(self):
        # computing the index of closest coordinates in waypoint
        closest_idx = self.get_closest_waypoint_idx()
        
        # computing the index of farthest coordinates in waypoints which 
        # is either within our LOOKAHEAD_WPS limit or LOOKAHEAD_WPS is farthest
        farthest_idx = min(closest_idx + LOOKAHEAD_WPS, len(self.base_waypoints.waypoints))
        
        # if farthest point is not in our LOOKAHEAD_WPS, then accelerate
        # otherwise stop at the required place
        if self.stop_line_wp_idx == -1 or self.stop_line_wp_idx >= farthest_idx or self.stop_line_wp_idx < closest_idx + 1:
            return self.accelerate_to_target_velocity(closest_idx, farthest_idx)
        else:
            return self.stop_at_stop_line(closest_idx, farthest_idx)

    def get_closest_waypoint_idx(self):
        # Extracing the x and y values from pose(current postion) message
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Quering the closest coordinates to current pose from shortlisted waypoint
        closest_idx = self.waypoints_tree.query([x,y],1)[1]
     
        # Check if closest is ahead or behind the vehicle
        closest_waypoint = self.base_waypoints.waypoints[closest_idx]
        prev_waypoint = self.base_waypoints.waypoints[
            (closest_idx-1) if closest_idx > 0 else (len(self.base_waypoints.waypoints)-1)]
        
        closest_coord = [closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y]
        prev_coord = [prev_waypoint.pose.pose.position.x, prev_waypoint.pose.pose.position.y]
                
        # Equation for hyperplane through closest coordinates
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        
        if val>0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)            
        return closest_idx
    
    #def accelerate_to_target_velocity(self, closest_coord, farthest_idx):
    def accelerate_to_target_velocity(self, closest_idx, farthest_idx):
        final_waypoints = []
        for i in LOOKAHEAD_WPS_SAMPLES:
            idx = closest_idx + i
            # gathering the waypoints up to fathest_idx
            if idx < farthest_idx:
                wp = self.base_waypoints.waypoints[idx]
                final_waypoints.append(wp)
                
        return final_waypoints
    
    def stop_at_stop_line(self, closest_idx, farthest_idx):
        final_waypoints = []
        stop_idx = max(self.stop_line_wp_idx - STOPPING_WPS_BEFORE, closest_idx)
        target_wp = self.base_waypoints.waypoints[stop_idx]
        dist = 0.0
        
        for i in LOOKAHEAD_WPS_SAMPLES[::-1]:
            idx = closest_idx + i
            if idx < farthest_idx:
                wp = self.base_waypoints.waypoints[idx]
                p = Waypoint()
                p.pose = wp.pose
                vel = 0.0
                
                if idx < stop_idx:
                    # Calculating the distance from the stop line to current position
                    dist += math.sqrt((target_wp.pose.pose.position.x - wp.pose.pose.position.x)**2 +
                                     (target_wp.pose.pose.position.y - wp.pose.pose.position.y)**2)
                    
                    # Reducing the velocity according to the max acceleration
                    vel = math.sqrt(2*MAX_DECEL*dist)
                    
                    if vel < 1.0:
                        vel = 0.0
                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                final_waypoints.insert(0,p)
        return final_waypoints
             
    
    #def publish_waypoints(self, closest_idx):
    def publish_waypoints(self, final_waypoints):
        lane = Lane()
        lane.header = self.base_waypoints.header
        #lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        lane.waypoints = final_waypoints
        self.final_waypoints_pub.publish(lane)
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        #if not self.waypoints_2d:
        self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                             for waypoint in waypoints.waypoints] 
        self.waypoints_tree = KDTree(self.waypoints_2d)  
                   
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_line_wp_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    #def distance(self, waypoints, wp1, wp2):
    #    dist = 0
    #    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #    for i in range(wp1, wp2+1):
    #        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #        wp1 = i
    #    return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
