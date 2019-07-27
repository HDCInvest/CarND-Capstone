#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

from scipy.spatial import KDTree
import numpy as np
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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        #rospy.spin()
        self.loop() 
    
    # Loop function is used so that we have control over publishing frequency
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                #Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx() #Get the closest waypoint in front of car
                self.publish_waypoints(closest_waypoint_idx)  # Publish the 200 waypoints in front
            rate.sleep()
    
    def get_closest_waypoint_idx(self):
        """
        Finds the closest waypoint infront of the car and  returns the id of it
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        #Check whether the closest waypoint is ahead or behind the car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1] 
        #Equation for the hyperplane through the closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        #If dot product is positive waypoint is behind the car and vise versa
        val = np.dot(cl_vect-prev_vect,pos_vect-cl_vect)
        if val > 0 : # waypoint is behind us
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d) # Take the next one

        return closest_idx

    def publish_waypoints(self,closest_idx):
        """
        Slices the base waypoints by cosidering the closest waypoint in front 
        of the car and adding LOOKAHEAD_WPS number of  waypoints from there. 
        Then it publishes the sliced base waypoints.
        Here we have set LOOKAHEAD_WPS = 200
        """
        lane = Lane()
        lane.header = self.base_waypoints.header  #This is not used anywhere
        lane.waypoints = self.base_waypoints.waypoints[closest_idx : closest_idx+LOOKAHEAD_WPS]  #Consider the next 200 waypoints
        self.final_waypoints_pub.publish(lane) 


    def pose_cb(self, msg):
        # TODO: Implement
        #This stores the car pose and this get called very frequently, like 50hz
        self.pose = msg  

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #Store the base waypoints and base waypoints doesn't change
        self.base_waypoints = waypoints
        #To find the closed waypoint infront of the car we use KDTree
        #KDTree is efficient way to look up the closest point in space
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

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
