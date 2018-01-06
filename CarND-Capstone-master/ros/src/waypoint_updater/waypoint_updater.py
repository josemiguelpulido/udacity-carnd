#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', PoseStamped, self.traffic_waypoint_cb)
        #rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_waypoint_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []
        self.final_waypoints = []
        self.rate = rospy.Rate(50) # Hz
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.logwarn(msg.pose.position.x)

        # get car's current position, orientation and angle
        car_position = msg.pose.position
        car_orientation =  msg.pose.orientation
        car_theta = math.atan2(car_orientation.y, car_orientation.x)

        # check if we have received waypoints
        if not self.base_waypoints:
            rospy.logwarn("base waypoints not yet acquired. Skipping received pose information")

        # determine closest waypoint
        closest_wp = -1
        closest_distance = 1000 #initial large number

        # define distance in 3D and heading
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        hl = lambda a, b: math.atan2((a.y-b.y),(a.x-b.x))

        # look for closest waypoint
        for i in range(len(self.base_waypoints)):

            # only consider waypoints ahead
            heading = hl(car_orientation, self.base_waypoints[i].pose.pose.orientation)
            angle = abs(heading - car_theta)
            if angle > math.pi/4:
                continue

            dist = dl(car_position, self.base_waypoints[i].pose.pose.position)
            if dist < closest_distance:
                closest_distance = dist
                closest_wp = i

        # assume waypoints are ordered and return the closest LOOKAHEAD_WPS waypoints
        self.final_waypoints = self.base_waypoints[closest_wp:]
        if len(self.final_waypoints) > LOOKAHEAD_WPS:
            self.final_waypoints = self.final_waypoints[:LOOKAHEAD_WPS]

        # publish resulting waypoints
        lane = Lane()
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)
        self.rate.sleep() # !?
        return
            
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        return

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x
    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
