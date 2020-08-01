#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

class AckermannModel(object):
	def __init__(self, wheelbase):
		self.L = wheelbase

	def path_radius(self, steering_angle):
		return self.L / np.tan(steering_angle)

	def yaw_rate(self, steering_angle, speed):
		if steering_angle == 0.0:
			return 0.0
		return speed / self.path_radius(steering_angle)

	def dx(self, speed, dt, steering_angle):
		if steering_angle == 0.0:
			return speed * dt
		R = self.path_radius(steering_angle)
		d = dt*speed
		dx = R*np.sin(d/R)
		return dx

	def dy(self, speed, dt, steering_angle):
		if steering_angle == 0.0:
			return 0.0
		R = self.path_radius(steering_angle)
		d = dt*speed
		dy = R*(1.0 - np.cos(d/R))
		return dy

	def steering_angle(self, point):
		if point[0] >= 0.0:
			theta = np.arctan(point[1]/point[0])
		else:
			theta = np.arctan(abs(point[0])/point[1]) + np.sign(point[1])*np.pi/2.0

		return np.arctan(2.0*self.L*np.sin(theta)/np.linalg.norm(point))

	def steering_angle_polar(self, polar_point):
		theta = polar_point[1]
		radius = polar_point[0]
		return np.arctan(2.0*self.L*np.sin(theta)/radius)

class following_path:
    global steering_angle_last
    steering_angle_last = 0
    
    def __init__(self):
        self.current_pose = rospy.Subscriber('odom111', Odometry, self.callback_read_current_position, queue_size=1)
        self.Pose = []
        self.path_pose = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.callback_read_path, queue_size=1)
        # self.path_pose2 = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.callback_read_path2, queue_size=1)
        self.path_pose2 = rospy.Subscriber('/move_base/GlobalPlanner_lhj/plan', Path, self.callback_read_path2, queue_size=1)
        self.path_info = []
        self.Goal = []
        self.path_info2 = []
        self.Goal2 = []
        self.NextPoint = []
        self.navigation_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.MAX_VELOCITY = 0.5
        self.MIN_VELOCITY = 0
        self.max_angle = 3.5 # 0.85
        self.k = -0.1 #0.205
        self.steering_velocity = 1
        self.jerk = 0.0
        self.acceleration = 0.0
        self.lookahead_k=0.5
        self.init_markers_goal() 
        self.init_markers_current() 
        self.wheelbase_length  =  0.31
        self.model = AckermannModel(self.wheelbase_length)

    def init_markers_goal(self):  
        # Set up our waypoint markers  
        marker_scale = 0.15  
        marker_lifetime = 0 # 0 is forever  
        marker_ns = 'waypoints'  
        marker_id = 0  
        marker_color = {'r': 238.0, 'g': 118, 'b': 0.0, 'a': 1.0}  
          
        # Define a marker publisher.  
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)  
          
        # Initialize the marker points list.  
        self.markers = Marker()  
        self.markers.ns = marker_ns  
        self.markers.id = marker_id  
        self.markers.type = Marker.SPHERE_LIST  
        self.markers.action = Marker.ADD  
        self.markers.lifetime = rospy.Duration(marker_lifetime)  
        self.markers.scale.x = marker_scale  
        self.markers.scale.y = marker_scale  
        self.markers.color.r = marker_color['r']  
        self.markers.color.g = marker_color['g']  
        self.markers.color.b = marker_color['b']  
        self.markers.color.a = marker_color['a']  
          
        self.markers.header.frame_id = 'map'  
        self.markers.header.stamp = rospy.Time.now()  
        self.markers.points = list() 
    
    def init_markers_current(self):  
        # Set up our waypoint markers  
        marker_scale = 0.15  
        marker_lifetime = 0 # 0 is forever  
        marker_ns = 'current_pose_points'  
        marker_id = 1  
        marker_color = {'r': 128, 'g': 0, 'b': 128.0, 'a': 1.0}  
          
        # Define a marker publisher.  
        self.markers2_pub = rospy.Publisher('current_pose_point_markers', Marker)  
          
        # Initialize the marker points list.  
        self.markers2 = Marker()  
        self.markers2.ns = marker_ns  
        self.markers2.id = marker_id  
        self.markers2.type = Marker.SPHERE_LIST  
        self.markers2.action = Marker.ADD  
        self.markers2.lifetime = rospy.Duration(marker_lifetime)  
        self.markers2.scale.x = marker_scale  
        self.markers2.scale.y = marker_scale  
        self.markers2.color.r = marker_color['r']  
        self.markers2.color.g = marker_color['g']  
        self.markers2.color.b = marker_color['b']  
        self.markers2.color.a = marker_color['a']  
          
        self.markers2.header.frame_id = 'map'  
        self.markers2.header.stamp = rospy.Time.now()  
        self.markers2.points = list() 

    def callback_read_path(self, data):
        # Organize the pose message and only ask for (x,y) and orientation
        # Read the Real time pose message and load them into path_info
        self.path_info = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            self.path_info.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal = list(self.path_info[-1]) # Set the last pose of the local path as goal location

    def callback_read_path2(self, data):
        # Organize the pose message and only ask for (x,y) and orientation
        # Read the Real time pose message and load them into path_info
        self.path_info2 = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            self.path_info2.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal2 = list(self.path_info2[-1]) # Set the last pose of the global path as goal location

    def callback_read_current_position(self, data):
        global steering_angle_last
        steering_angle_last = 0
        angle_control = 0
        VELOCITY = 0
        angle = 0

        if not len(self.path_info2) == 0:
            # Read the current pose of the car from particle filter
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w

            # Read the path information to path_point list
            if y>=1 or x>=28:
                path_points_x = [float(point[0]) for point in self.path_info2]
                path_points_y = [float(point[1]) for point in self.path_info2]
                path_points_w = [float(point[2]) for point in self.path_info2]
                print('global planner')
            else:
                path_points_x = [float(point[0]) for point in self.path_info]
                path_points_y = [float(point[1]) for point in self.path_info]
                path_points_w = [float(point[2]) for point in self.path_info]
                print('local planner')

            # Convert the quaternion angle to eular angle
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.Pose = [float(x), float(y), float(yaw)]

            # 2. Find the path point closest to the vehichle tat is >= 1 lookahead distance from vehicle's current location.
            dist_array = np.zeros(len(path_points_x))

            for i in range(len(path_points_x)):
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]), (x,y))
            
            goal = np.argmin(dist_array) # Assume the closet point as the goal point at first
            LOOKAHEAD_DISTANCE = self.lookahead_k*VELOCITY + 0.3
            goal_array = np.where((dist_array < (LOOKAHEAD_DISTANCE + 0.3)) & (dist_array > (LOOKAHEAD_DISTANCE - 0.3)))[0]
            for id in goal_array:
                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
                if abs(diff_angle) < np.pi/4: # Check if the one that is the cloest to the lookahead direction
                    goal = id
                    break

            # draw the goal point (green)
            p = Point()
            p.x = path_points_x[goal]
            p.y = path_points_y[goal]
            p.z = 0
            self.markers.points.append(p)
            self.marker_pub.publish(self.markers)

            # draw the pose point (purple)
            q = Point()
            q.x = x
            q.y = y
            q.z = 0
            self.markers2.points.append(q)
            self.markers2_pub.publish(self.markers2)

            # Read the next point to go
            self.NextPoint = [path_points_x[goal], path_points_y[goal], path_points_w[goal]]

            # # 3. Calculate the curvature = 1/r = 2x/l^2
            # # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
            
            angle_to_go = []
            if goal>=(len(dist_array)-3):
                Ls = dist_array[goal]
                diff_angles = path_points_w[goal] - yaw # Find the turning angle
                rs = Ls/(2*math.sin(diff_angles)) # Calculate the turning radius
                angle_raw = math.atan(0.335/rs) # Find the wheel turning radius
            else:
                for i in range(0,3):
                    Ls = dist_array[goal+i]
                    diff_angles = path_points_w[goal+i] - yaw # Find the turning angle
                    rs = Ls/(2*math.sin(diff_angles)) # Calculate the turning radius
                    angles = math.atan(0.335/rs) # Find the wheel turning radius
                    angle_to_go.append(angles)
                angle_raw = 0.5*angle_to_go[0] + 0.3*angle_to_go[1] + 0.2*angle_to_go[2]# + 0.15*angle_to_go[3] + 0.05*angle_to_go[4] 0.5,0.3,0.2
            
            angle_error = self.determine_steering_angle(self.Pose, self.NextPoint)
            angle = angle_raw + 0.2215*angle_error
            angle = np.clip(angle, -self.max_angle-0.35, self.max_angle+0.35)

            diff_angle_array = []
            if goal>=(len(dist_array)-3):
                max_angle = angle
            else:
                # points after goal
                for i in range(0,3):
                    diff_angle_id = path_points_w[goal+i] - yaw
                    diff_angle_array.append(diff_angle_id)
                max_angle = max(diff_angle_array)
            # angle = (0 if abs(max_angle)<=0.07 else angle)
            if y<=0.9 and abs(max_angle)<=0.1:
                angle = 0
            elif y>=1.1 and abs(max_angle)<=0.2:
                angle = 0
            else:
                angle = angle

            #4. Calculate the velocity to control
            diff_control_angle_array = []
            # ponits after goal
            for i in range(0,3):
                veclocity_control = self.dist((path_points_x[goal+i], path_points_y[goal+i]), (x,y))
                control_L = dist_array[goal+i]
                diff_control_angle = path_points_w[goal+i] - yaw # Find the turning angle
                control_r = control_L/(2*math.sin(diff_control_angle)) # Calculate the turning radius
                control_angle = math.atan(0.335/control_r) # Find the wheel turning radius
                diff_control_angle_array.append(control_angle)
            angle_control = np.mean(diff_control_angle_array)

            # angle control speed
            if y<=1.1:
                VELOCITY = self.speed_control_go(angle_control, self.MIN_VELOCITY, self.MAX_VELOCITY)
            else:
                VELOCITY = self.speed_control_back(angle_control, self.MIN_VELOCITY, self.MAX_VELOCITY)

            #Special points
            length_array = []
            length0 = self.dist((path_points_x[goal], path_points_y[goal]), (35.3,1.15))
            length7 = self.dist((path_points_x[goal], path_points_y[goal]), (7.92,2.92))
            length6 = self.dist((path_points_x[goal], path_points_y[goal]), (7.86,-3.06))
            length5 = self.dist((path_points_x[goal], path_points_y[goal]), (15.8,-2.65))
            length4 = self.dist((path_points_x[goal], path_points_y[goal]), (7.92,-1.12))
            length3 = self.dist((path_points_x[goal], path_points_y[goal]), (18.49,2.26))
            length2 = self.dist((path_points_x[goal], path_points_y[goal]), (34.28,1.96))
            length1 = self.dist((path_points_x[goal], path_points_y[goal]), (15.835,4.575))

            if length0<=2 or length1<=2 or length2<=2 or length3<=2 or length4<=2 or length5<=2 or length6<=2 or length7<=2:
                VELOCITY = VELOCITY - 0.17

            # Write the Velocity and angle data into the ackermann message
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = VELOCITY
            ackermann_control.drive.steering_angle = angle
            ackermann_control.drive.steering_angle_velocity = self.steering_velocity   
        else:
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
        
        print('v:',VELOCITY , 'angle:',angle)
        self.navigation_input.publish(ackermann_control)
    
    # Computes the Euclidean distance between two 2D points p1 and p2
    def dist(self, p1, p2):
	    try:
		    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
	    except:
		    return 0.5

    # Compute the angle between car direction and goal direction
    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)
        sin_ang = LA.norm(np.cross(v1, v2)) 
        return np.arctan2(sin_ang, cos_ang) 

    # Control the speed of the car within the speed limit
    def speed_control_go(self, angle, MIN_VELOCITY, MAX_VELOCITY):
        # Assume the speed change linearly with respect to yaw angle
        if abs(angle)<=0.1:
            Velocity = 0.6
        else:
            Velocity = self.k * abs(angle) + MAX_VELOCITY + 0.1
        return Velocity

    def speed_control_back(self, angle, MIN_VELOCITY, MAX_VELOCITY):
        # Assume the speed change linearly with respect to yaw angle
        if abs(angle)>=2.3 and abs(angle)<4:
            Velocity = 0.25
        elif abs(angle)>=2 and abs(angle)<2.3:
            Velocity = 0.35
        elif abs(angle)>=1.7 and abs(angle)<2.0:
            Velocity = 0.4
        elif abs(angle)<=0.1:
            Velocity = 0.6
        else:
            Velocity = self.k * abs(angle) + MAX_VELOCITY + 0.05
        return Velocity

    def determine_steering_angle(self, pose, lookahead_point):
        global steering_angle_last
        rot = np.matrix([[np.cos(-pose[2]), -np.sin(-pose[2])], [np.sin(-pose[2]), np.cos(-pose[2])]]) # [ [a,b] , [c,d] ]
        delta = np.matrix([[lookahead_point[0] - pose[0]],[lookahead_point[1] - pose[1]]])
        local_delta = (rot*delta).transpose()
        local_delta = np.array([local_delta[0,0], local_delta[0,1]])
        angle_error = self.model.steering_angle(local_delta)
        return angle_error
    
if __name__ == "__main__":

    rospy.init_node("pursuit_path")
    following_path()
    rospy.spin()
