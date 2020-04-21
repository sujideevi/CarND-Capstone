#!/usr/bin/env python
import rospy
import tf
import cv2
import yaml
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier


import math
import numpy as np
from points_organizer import PointsOrganizer

STATE_COUNT_THRESHOLD = 3
MAX_DETECTION_DIST = 100 


class TLDetector(object):
    def __init__(self):
        #rospy.init_node('tl_detector')
        rospy.init_node('tl_detector',log_level=rospy.DEBUG)
        
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoints_organizer = None
        self.stop_line_organizer = None
        
        # Loading the stopline positions and camera image size parameters from config file
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
              
        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_organizer = PointsOrganizer(
            [[stop_line[0],stop_line[1]] for stop_line in self.stop_line_positions])
        
        self.has_image = False
        self.camera_image = None
        self.image_counter = 0
        self.image_classifier_is_ready = False
                
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size = 1, 
                               buff_size = 100*1024*1024, tcp_nodelay=True)
        
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        #self.listener = tf.TransformListener()
        self.image_classifier_is_ready = True
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_organizer = PointsOrganizer(
        [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints])

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        rospy.logdebug('Image processed, light_wp: {}, state: {}.'.format(light_wp, state))
        
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.last_state == TrafficLight.YELLOW and state == TrafficLight.GREEN:
            self.upcoming_red_light_pub.publish(Int32(light_up))
            return
        
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            rospy.logdebug('State recorded, previous: {}, new: {}.'.format(self.last_state, self.state))
            self.last_state = self.state
            #light_wp = light_wp if state == TrafficLight.RED else -1
            light_wp = light_wp if state != TrafficLight.GREEN else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
     
        
    #def get_closest_waypoint(self, pose):
    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        '''if(not self.has_image):
            self.prev_light_loc = None
            return False
        '''
        if not self.image_classifier_is_ready or not self.has_image:
            return TrafficLight.UNKNOWN
        
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        if self.pose and self.waypoints_organizer and self.stop_line_organizer and self.lights:
            closest_waypoint_idx = self.waypoints_organizer.get_closest_point_idx(
                self.pose.pose.position.x, self.pose.pose.position.y, look_mode='AHEAD')
            closest_waypoint = self.waypoints.waypoints[closest_waypoint_idx]
            #car_position = self.get_closest_waypoint(self.pose.pose)
            #car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            # Getting the closest stop line index ahead of vehicle
            closest_stop_line_idx  = self.stop_line_organizer.get_closest_point_idx(
                closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y, look_mode='AHEAD')
            
            if closest_stop_line_idx is not None:
                closest_light = self.lights[closest_stop_line_idx]
                
                dist_to_light = math.sqrt((self.pose.pose.position.x - closest_light.pose.pose.position.x)**2+
                                         (self.pose.pose.position.y - closest_light.pose.pose.position.y)**2)
                
                # classify and Publishing traffic light only if it is within MAX_DETECTION_DIST
                if dist_to_light > MAX_DETECTION_DIST:
                    return -1, TrafficLight.UNKNOWN
                
                # Getting stop line associated to upcoming traffic light
                closest_stop_line = self.stop_line_positions[closest_stop_line_idx]
                
                # Getting waypoints closest to stopline
                stop_waypoints_idx =  self.waypoints_organizer.get_closest_point_idx(
                    closest_stop_line[0], closest_stop_line[1], look_mode = 'AHEAD')
                
                state = self.get_light_state(closest_light)
                return stop_waypoints_idx, state
                
            '''
            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in ennumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0],line[1])

                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx

                if d>=0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
            '''
            
        '''
        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        self.waypoints = None
        '''
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
