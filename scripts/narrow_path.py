#!/usr/bin/env python 
import rospy
import math

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

#front x +
#left y +
class narrow_path:
	def __init__(self):
		rospy.init_node('narrow_path', anonymous=True)
				
		self.pub = rospy.Publisher('ackermann', AckermannDriveStamped, queue_size=10)		

		self.wayPoint = Point(1,0,0)
		
		#self.control_factor = rospy.get_param("/narrow_path/control_factor", 104)
		self.right_steer_scale = rospy.get_param("/narrow_path/right_steer_scale", 2.0)	
                self.throttle = rospy.get_param("/narrow_path/throttle",4)
		
		self.start_distance = rospy.get_param("/narrow_path/start_distance", 1.0)
		self.end_distance = rospy.get_param("/narrow_path/end_distance", 1.0)
		self.approach_left_offset = rospy.get_param("/narrow_path/approach_left_offset", 7)

		self.approach_flag = False
		self.start_flag = True
		self.steer_angle = 0.0
		self.finish_flag = False

	def updateParam(self):
		#self.control_factor = rospy.get_param("/narrow_path/control_factor")
		self.right_steer_scale = rospy.get_param("/narrow_path/right_steer_scale")	
		#print("control_factor: " + str(self.control_factor))	
		print("right_steer_scale: " + str(self.right_steer_scale))
	
	def execute_cb(self):
		rospy.loginfo("narrow execute_cb")
		self.sub = rospy.Subscriber('narrow_path_raw_obstacles', Obstacles, self.obstacles_cb)
		self.sub2 = rospy.Subscriber('narrow_path_approach_raw_obstacles', Obstacles, self.approach_cb)
		self.sub3 = rospy.Subscriber('narrow_path_escape_raw_obstacles', Obstacles, self.escape_cb)

		

	def narrow_pathing(self, data):
		self.updateParam()
		print("during narrow mission")
		print("#######################################################")		
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = self.throttle		
		if len(data.segments) >= 2:
			 
			x_center = 0
			y_center = 0
			#find wayPoint 
		
			for segment_data in data.segments:
				x_center = x_center + segment_data.first_point.x
				x_center = x_center + segment_data.last_point.x
				y_center = y_center + segment_data.first_point.y
				y_center = y_center + segment_data.last_point.y
		 		

			print(len(data.segments))
				
			x_center = x_center/len(data.segments)
			y_center = y_center/len(data.segments)
			self.wayPoint = Point(x_center,y_center,0)
			print(self.wayPoint)
			#if detect segment, up start signal and start mission
			self.start_signal = 1			
			self.steer_angle = math.atan((self.wayPoint.y/self.wayPoint.x))
			acker_data.drive.steering_angle = int((-self.steer_angle/math.pi)*104 )		
                else:
			acker_data.drive.steering_angle = int((-self.steer_angle/math.pi)*104 )		
			
		
		if (acker_data.drive.steering_angle > 26):
			acker_data.drive.steering_angle = 26
		elif (acker_data.drive.steering_angle < -26):
			acker_data.drive.steering_angle = -26
		print("speed : " + str(acker_data.drive.speed))
		print("steering : " + str(acker_data.drive.steering_angle))

		if not self.finish_flag:
			self.pub.publish(acker_data)

	def narrow_pathing_approach(self, data):
		self.updateParam()
	
		x_center = 0
		y_center = 0

		nearest_x = 50.0
		nearest_y_1 = -100.0
		nearest_y_2 = 100.0
		for segment_data in data.segments:
			if nearest_x > segment_data.first_point.x:
				nearest_x = segment_data.first_point.x
			if nearest_x > segment_data.last_point.x:
				nearest_x = segment_data.last_point.x
			if segment_data.first_point.y < 0:
				if nearest_y_1 < segment_data.first_point.y:
					nearest_y_1 = segment_data.first_point.y
				if nearest_y_1 < segment_data.last_point.y:
					nearest_y_1 = segment_data.last_point.y
			if segment_data.first_point.y >=0:
				if nearest_y_2 > segment_data.first_point.y:
					nearest_y_2 = segment_data.first_point.y
				if nearest_y_2 > segment_data.last_point.y:
					nearest_y_2 = segment_data.last_point.y

			
			
		x_center = nearest_x
		y_center = nearest_y_1 + nearest_y_2

		x_center = x_center
		y_center = y_center/2
		self.wayPoint = Point(x_center,y_center,0)
       
		print(self.wayPoint)
		#if detect segment, up start signal and start mission
		self.start_signal = 1
		
		print("#######################################################")
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = self.throttle		
		steer_angle = math.atan(self.wayPoint.y/self.wayPoint.x)
		print("steer_angle : " + str(steer_angle))
		acker_data.drive.steering_angle = int((-steer_angle/math.pi) * 104)
		if (acker_data.drive.steering_angle > 26):
			acker_data.drive.steering_angle = 26
		elif (acker_data.drive.steering_angle < -26):
			acker_data.drive.steering_angle = -26
		print("speed : " + str(acker_data.drive.speed))
		print("steering : " + str(acker_data.drive.steering_angle))

		if not self.finish_flag:
			self.pub.publish(acker_data)

	def farthestpoint(self, data):
		x_center = 0
		y_center = 0

		far_x = 0
		far_y_1 = 0
		far_y_2 = 0
		for segment_data in data.segments:
			if far_x < segment_data.first_point.x:
				far_x = segment_data.first_point.x
			if far_x < segment_data.last_point.x:
				far_x = segment_data.last_point.x
			if segment_data.first_point.y < 0:
				if far_y_1 > segment_data.first_point.y:
					far_y_1 = segment_data.first_point.y
				if far_y_1 > segment_data.last_point.y:
					far_y_1 = segment_data.last_point.y
			if segment_data.first_point.y >= 0:
				if far_y_2 < segment_data.first_point.y:
					far_y_2 = segment_data.first_point.y
				if far_y_2 < segment_data.last_point.y:
					far_y_2 = segment_data.last_point.y
		
		x_center = far_x
		y_center = far_y_1 + far_y_2

		x_center = x_center
		y_center = y_center/2
		wayPoint = Point(x_center, y_center ,0)
       
		print(wayPoint)
		return wayPoint
	

	def approach_cb(self, data):
		if self.start_flag == True:
			print("approache_cb")
			self.narrow_pathing_approach(data)

			nearest_x = 100.0
			for segment_data in data.segments:
				if nearest_x > segment_data.first_point.x:
					nearest_x = segment_data.first_point.x
				if nearest_x > segment_data.last_point.x:
					nearest_x = segment_data.last_point.x
			

			if nearest_x < self.start_distance:
				self.start_flag = False
				self.approach_flag = True

	def obstacles_cb(self, data):
		if self.approach_flag == True:
			print("obstacle_cb")
			
			self.narrow_pathing(data)
			
			farthest_x = 0
			for segment_data in data.segments:
				if farthest_x < segment_data.first_point.x:
					farthest_x = segment_data.first_point.x
				if farthest_x < segment_data.last_point.x:
					farthest_x = segment_data.last_point.x

			
			if farthest_x < self.end_distance:
				self.finish_flag = True


	def calc_distance(self, point1, point2):
        	distance = (point1.x- point2.x)**2 + (point1.y - point2.y)**2
		return distance

	def escape_cb(self, data):
		print("escape")
		
		acker_data = AckermannDriveStamped()	
		finish_point = self.farthestpoint(data)	

		print("finish_ point : " + str(finish_point))
		if self.finish_flag == True:
			print("true")
			if abs(self.calc_distance(self.wayPoint, finish_point)) < 0.7: 
				self.approach_flag = False
				print("true12###########~!~!~!~!~!~!~!~!~!~!#######")
				acker_data.drive.speed = 0
				self.pub.publish(acker_data)
				print("end")
		
if __name__ == '__main__':
	try:
		narrow_mission = narrow_path()
		narrow_mission.execute_cb()
		
		rospy.spin()
	except rospy.ROSInterruptException:
		print(error)
pass		
				
