#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

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

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        # determine closest waypoint
            
        # get car's current position, orientation and angle
        car_position = pose.position
        car_orientation =  pose.orientation
        car_theta = math.atan2(car_orientation.y, car_orientation.x)

        # determine closest waypoint
        closest_wp = -1
        closest_distance = 1000 #initial large number

        # define distance and heading
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        hl = lambda a, b: math.atan2((a.y-b.y),(a.x-b.x))

        # look for closest waypoint
        for i in range(len(self.waypoints)):

            # only consider waypoints ahead
            heading = hl(car_orientation, self.waypoints[i].pose.pose.orientation)
            angle = abs(heading - car_theta)
            if angle > math.pi/4:
                continue

            # determine closest candidate
            dist = dl(car_position, self.waypoints[i].pose.pose.position)
            if dist < closest_distance:
                closest_distance = dist
                closest_wp = i

        return closest_wp

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

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

        # check if we have received waypoints
        if self.waypoints == None:
            rospy.logwarn("tl_detector: waypoints not yet acquired. Skipping received image information")
            return -1, TrafficLight.UNKNOWN

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        #rospy.logwarn(stop_line_positions)
        
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)        
        dl = lambda a, b, c, d: math.sqrt((a-b)**2 + (c-d)**2)
        closest_light = -1
        closest_light_distance = 100000
        for i in range(len(stop_line_positions)):
            light_x = stop_line_positions[i][0]
            light_y = stop_line_positions[i][1]
            car_x = self.waypoints[car_position].pose.pose.position.x
            car_y = self.waypoints[car_position].pose.pose.position.y
            dist = dl(car_x, light_x, car_y, light_y)
            if dist < closest_light_distance:
                closest_light = i
                closest_light_distance = dist
                
        # define traffic light position, reusing cars' orientation
        light = TrafficLight()
        light.pose.pose.position.x = stop_line_positions[closest_light][0]
        light.pose.pose.position.y = stop_line_positions[closest_light][1]
        light.pose.pose.orientation = self.waypoints[car_position].pose.pose.orientation

        # find closest waypoint to traffic light
        light_wp = self.get_closest_waypoint(light.pose.pose)
        
        if light:
            #state = self.get_light_state(light)
            state = TrafficLight.RED
            rospy.logwarn("tl_detectot: sending light wp " + str(light_wp) + " and state "+ str(state))
            
            return light_wp, state
        
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
