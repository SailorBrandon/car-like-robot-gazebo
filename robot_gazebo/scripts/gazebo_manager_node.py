#!/usr/bin/env python3

import math
import rospy
import tf
from iss_manager.msg import State, ObjectDetection3D, ObjectDetection3DArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from gazebo_msgs.msg import ModelStates
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class GazeboManagerNode:
    def __init__(self) -> None:
        self._gt_ego_odom_pub = rospy.Publisher("gazebo_bridge/ego_odom", Odometry, queue_size=1)
        self._ego_state_pub = rospy.Publisher("gazebo_bridge/ego_state_estimation", State, queue_size=1)
        self._object_detection_pub = rospy.Publisher("gazebo_bridge/object_detection", ObjectDetection3DArray, queue_size=1)
        
        self._gazebo_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._gazebo_states_callback)
        self._ego_vehicle_name = rospy.get_param("robot_name", "ego_vehicle")
        
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._ego_tf_pub_timer = rospy.Timer(rospy.Duration(0.1), self._ego_tf_pub_callback)
        
        self._ego_odom = None
        self._tf_listener = tf.TransformListener()
        self._ego_state_pub_timer = rospy.Timer(rospy.Duration(0.1), self._ego_state_pub_callback)
        
        self._ack_sub = rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped, self.ack_callback)
        self._cmd_vel_sub = rospy.Subscriber("gazebo_bridge/cmd_vel", Twist, self.cmd_vel_callback)

        self._pub_vel_left_rear_wheel = rospy.Publisher("left_rear_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_vel_right_rear_wheel = rospy.Publisher("right_rear_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_vel_left_front_wheel = rospy.Publisher("left_front_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_vel_right_front_wheel = rospy.Publisher("right_front_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_pos_left_steering_hinge = rospy.Publisher("left_steering_hinge_position_controller/command", Float64, queue_size=1)
        self._pub_pos_right_steering_hinge = rospy.Publisher("right_steering_hinge_position_controller/command", Float64, queue_size=1)
        self._fac = 31.25
            
    def _ego_tf_pub_callback(self, event):
        if self._ego_odom is None:
            return
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = self._ego_odom.pose.pose.position.x
        tf_msg.transform.translation.y = self._ego_odom.pose.pose.position.y
        tf_msg.transform.translation.z = self._ego_odom.pose.pose.position.z
        tf_msg.transform.rotation.x = self._ego_odom.pose.pose.orientation.x
        tf_msg.transform.rotation.y = self._ego_odom.pose.pose.orientation.y
        tf_msg.transform.rotation.z = self._ego_odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = self._ego_odom.pose.pose.orientation.w
        self._tf_broadcaster.sendTransformMessage(tf_msg)
   
    def _gazebo_states_callback(self, msg):
        all_detections = ObjectDetection3DArray()
        self._ego_odom = Odometry()
        for i in range(len(msg.name)):
            if msg.name[i][:len("obstacle")] == "obstacle":
                detection = ObjectDetection3D()
                detection.header.stamp = rospy.Time.now()
                detection.header.frame_id = "map"
                detection.id = 0
                detection.score = 1.0
                detection.state.x = msg.pose[i].position.x
                detection.state.y = msg.pose[i].position.y
                ang = tf.transformations.euler_from_quaternion([msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w])
                detection.state.heading_angle = ang[2]
                detection.state.velocity = math.sqrt(msg.twist[i].linear.x**2 + msg.twist[i].linear.y**2)
                detection.state.acceleration = 0
                detection.bbox.size.x = 0.4
                detection.bbox.size.y = 0.2
                detection.bbox.size.z = 0.3
                detection.bbox.center.position.x = msg.pose[i].position.x
                detection.bbox.center.position.y = msg.pose[i].position.y
                detection.bbox.center.position.z = msg.pose[i].position.z
                detection.bbox.center.orientation.x = msg.pose[i].orientation.x
                detection.bbox.center.orientation.y = msg.pose[i].orientation.y
                detection.bbox.center.orientation.z = msg.pose[i].orientation.z
                detection.bbox.center.orientation.w = msg.pose[i].orientation.w
                all_detections.detections.append(detection)
            if msg.name[i] == "ego_vehicle":
                self._ego_odom.header.stamp = rospy.Time.now()
                self._ego_odom.header.frame_id = "odom"
                self._ego_odom.child_frame_id = "base_footprint"
                self._ego_odom.pose.pose.position.x = msg.pose[i].position.x
                self._ego_odom.pose.pose.position.y = msg.pose[i].position.y
                self._ego_odom.pose.pose.position.z = msg.pose[i].position.z
                self._ego_odom.pose.pose.orientation.x = msg.pose[i].orientation.x
                self._ego_odom.pose.pose.orientation.y = msg.pose[i].orientation.y
                self._ego_odom.pose.pose.orientation.z = msg.pose[i].orientation.z
                self._ego_odom.pose.pose.orientation.w = msg.pose[i].orientation.w
                self._ego_odom.twist.twist.linear.x = msg.twist[i].linear.x
                self._ego_odom.twist.twist.linear.y = msg.twist[i].linear.y
                self._ego_odom.twist.twist.linear.z = msg.twist[i].linear.z
                self._ego_odom.twist.twist.angular.x = msg.twist[i].angular.x
                self._ego_odom.twist.twist.angular.y = msg.twist[i].angular.y
                self._ego_odom.twist.twist.angular.z = msg.twist[i].angular.z
                self._gt_ego_odom_pub.publish(self._ego_odom)
        self._object_detection_pub.publish(all_detections)

    def _ego_state_pub_callback(self, event):
        if not self._tf_listener.canTransform("map", "base_footprint", rospy.Time(0)):
            rospy.logwarn("Cannot get transform from map to base_footprint")
            return
        self._tf_listener.waitForTransform("map", "base_footprint", rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self._tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        x, y = trans[0], trans[1]
        euler = tf.transformations.euler_from_quaternion(rot)
        heading_angle = euler[2]
        while (self._ego_odom is None):
            rospy.sleep(0.1)
        ego_state_msg = State()
        ego_state_msg.x = x
        ego_state_msg.y = y
        ego_state_msg.heading_angle = heading_angle
        ego_state_msg.velocity = math.sqrt(self._ego_odom.twist.twist.linear.x**2 + self._ego_odom.twist.twist.linear.y**2)
        self._ego_state_pub.publish(ego_state_msg)
        
    
    def ack_callback(self, msg):
        vel_left_rear_wheel = Float64()
        vel_right_rear_wheel = Float64()
        vel_left_front_wheel = Float64()
        vel_right_front_wheel = Float64()
        pos_left_steering_hinge = Float64()
        pos_right_steering_hinge = Float64()

        vel_left_rear_wheel.data = msg.drive.speed * self._fac
        vel_right_rear_wheel.data = msg.drive.speed * self._fac
        vel_left_front_wheel.data = msg.drive.speed * self._fac
        vel_right_front_wheel.data = msg.drive.speed * self._fac
        pos_left_steering_hinge.data = msg.drive.steering_angle
        pos_right_steering_hinge.data = msg.drive.steering_angle

        self._pub_vel_left_rear_wheel.publish(vel_left_rear_wheel)
        self._pub_vel_right_rear_wheel.publish(vel_right_rear_wheel)
        self._pub_vel_left_front_wheel.publish(vel_left_front_wheel)
        self._pub_vel_right_front_wheel.publish(vel_right_front_wheel)
        self._pub_pos_left_steering_hinge.publish(pos_left_steering_hinge)
        self._pub_pos_right_steering_hinge.publish(pos_right_steering_hinge)

    def cmd_vel_callback(self, msg):
        speed = msg.linear.x
        steering_angle = msg.angular.z
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = speed
        ack_msg.drive.steering_angle = steering_angle
        self.ack_callback(ack_msg)

if __name__ == "__main__":
    rospy.init_node("gazebo_manager_node", anonymous=True)
    gazebo_bridge = GazeboManagerNode()
    rospy.spin()