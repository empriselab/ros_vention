#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
	siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
	cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
	return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
	return math.atan2(math.sin(a - b), math.cos(a - b))


class OdomDifferentiator:
	def __init__(self):
		self.input_topic = rospy.get_param("~input_odom_topic", "/zed_mini/zed_node/odom")
		self.output_topic = rospy.get_param("~output_odom_topic", "/move_base/odom_feedback")

		self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
		self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=50)

		self.prev_msg = None
		self.prev_stamp = None
		self.prev_yaw = None

		rospy.loginfo("zed_pose_to_odom_feedback input=%s output=%s", self.input_topic, self.output_topic)

	def cb(self, msg):
		stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
		yaw = yaw_from_quat(msg.pose.pose.orientation)

		if self.prev_msg is None:
			self.prev_msg = msg
			self.prev_stamp = stamp
			self.prev_yaw = yaw

			out = Odometry()
			out.header = msg.header
			out.child_frame_id = msg.child_frame_id
			out.pose = msg.pose
			self.pub.publish(out)
			return

		dt = (stamp - self.prev_stamp).to_sec()
		if dt <= 0.0:
			return

		curr_p = msg.pose.pose.position
		prev_p = self.prev_msg.pose.pose.position

		dx = curr_p.x - prev_p.x
		dy = curr_p.y - prev_p.y
		dz = curr_p.z - prev_p.z
		dyaw = angle_diff(yaw, self.prev_yaw)

		vx_odom = dx / dt
		vy_odom = dy / dt
		vz_odom = dz / dt
		wz = dyaw / dt

		# Put translational velocity in base frame.
		c = math.cos(yaw)
		s = math.sin(yaw)
		vx = c * vx_odom + s * vy_odom
		vy = -s * vx_odom + c * vy_odom

		out = Odometry()
		out.header = msg.header
		out.child_frame_id = msg.child_frame_id
		out.pose = msg.pose
		out.twist.twist.linear.x = vx
		out.twist.twist.linear.y = vy
		out.twist.twist.linear.z = vz_odom
		out.twist.twist.angular.z = wz

		self.pub.publish(out)

		self.prev_msg = msg
		self.prev_stamp = stamp
		self.prev_yaw = yaw


def main():
	rospy.init_node("zed_pose_to_odom_feedback")
	OdomDifferentiator()
	rospy.spin()


if __name__ == "__main__":
	main()
