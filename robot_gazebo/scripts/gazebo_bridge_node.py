#!/usr/bin/env python3

import math
import rospy
import tf
from iss_manager.msg import State, ObjectDetection3D, ObjectDetection3DArray, ControlCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from gazebo_msgs.msg import ModelStates
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2 as pc2
from iss_manager.srv import SetGoal


class GazeboBridgeNode:
    def __init__(self) -> None:
        self._ego_vehicle_name = rospy.get_param("robot_name", "ego_vehicle")
        self._namespaces = rospy.get_namespace().replace("/", "") + "/"

        self._ego_odom_pub = rospy.Publisher(
            rospy.get_param("ego_odom_topic"), Odometry, queue_size=1)
        self._ego_state_pub = rospy.Publisher(
            rospy.get_param("ego_state_topic"), State, queue_size=1)
        self._object_detection_pub = rospy.Publisher(rospy.get_param(
            "object_detection_topic"), ObjectDetection3DArray, queue_size=1)
        self._world_frame = rospy.get_param("world_frame", "map")

        self._gazebo_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._gazebo_states_callback)

        self._tf_broadcaster = tf.TransformBroadcaster()
        self._ego_tf_pub_timer = rospy.Timer(
            rospy.Duration(0.1), self._ego_tf_pub_callback)

        self._ego_odom = None
        self._tf_listener = tf.TransformListener()
        self._ego_state_pub_timer = rospy.Timer(
            rospy.Duration(0.1), self._ego_state_pub_callback)

        self._control_cmd_sub = rospy.Subscriber(rospy.get_param(
            "control_command_topic"), ControlCommand, self._ctrl_cmd_callback)
        # self._control_cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self._ctrl_cmd_callback)
        self._pub_vel_left_rear_wheel = rospy.Publisher(
            "left_rear_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_vel_right_rear_wheel = rospy.Publisher(
            "right_rear_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_vel_left_front_wheel = rospy.Publisher(
            "left_front_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_vel_right_front_wheel = rospy.Publisher(
            "right_front_wheel_velocity_controller/command", Float64, queue_size=1)
        self._pub_pos_left_steering_hinge = rospy.Publisher(
            "left_steering_hinge_position_controller/command", Float64, queue_size=1)
        self._pub_pos_right_steering_hinge = rospy.Publisher(
            "right_steering_hinge_position_controller/command", Float64, queue_size=1)
        self._fac = 31.25

        self._velodyne_sub = rospy.Subscriber(rospy.get_param(
            "3d_lidar_topic"), PointCloud2, self._velodyne_callback)
        self._2d_laser_scan_pub = rospy.Publisher(
            rospy.get_param("2d_lidar_topic"), LaserScan, queue_size=1)

        rospy.wait_for_service('/gazebo/get_model_state', timeout=5)
        self._goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal_callback)
    
    def _goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        euler = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        heading_angle = euler[2]
        rospy.wait_for_service('planning/set_goal')
        try:
            set_goal = rospy.ServiceProxy('planning/set_goal', SetGoal)
            resp = set_goal(x, y, heading_angle)
            return resp.success
        except rospy.ServiceException as e:
            print("Planning service call failed: %s" % e)
            return False

    def _velodyne_callback(self, point_cloud_msg):
        laser_scan = LaserScan()
        laser_scan.header = point_cloud_msg.header
        laser_scan.angle_min = -math.pi
        laser_scan.angle_max = math.pi
        laser_scan.angle_increment = math.pi / 180
        laser_scan.time_increment = 0
        laser_scan.range_min = 0.0
        laser_scan.range_max = 100.0
        laser_scan.ranges = [float('Inf')] * 360
        for p in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            if p[2] < -0.15:
                continue
            angle = math.atan2(p[1], p[0])
            distance = math.sqrt(p[0]**2 + p[1]**2)
            index = int((angle - laser_scan.angle_min) /
                        laser_scan.angle_increment)
            if 0 <= index < len(laser_scan.ranges) and distance < laser_scan.ranges[index]:
                laser_scan.ranges[index] = distance
        self._2d_laser_scan_pub.publish(laser_scan)

    def _ego_tf_pub_callback(self, event):
        if self._ego_odom is None:
            return
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self._namespaces + "odom"
        tf_msg.child_frame_id = self._namespaces + "base_footprint"
        tf_msg.transform.translation.x = self._ego_odom.pose.pose.position.x
        tf_msg.transform.translation.y = self._ego_odom.pose.pose.position.y
        tf_msg.transform.translation.z = self._ego_odom.pose.pose.position.z
        tf_msg.transform.rotation.x = self._ego_odom.pose.pose.orientation.x
        tf_msg.transform.rotation.y = self._ego_odom.pose.pose.orientation.y
        tf_msg.transform.rotation.z = self._ego_odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = self._ego_odom.pose.pose.orientation.w
        if tf_msg.transform.rotation.x == 0 and tf_msg.transform.rotation.y == 0 and tf_msg.transform.rotation.z == 0 and tf_msg.transform.rotation.w == 0:
            return
        self._tf_broadcaster.sendTransformMessage(tf_msg)

    def _gazebo_states_callback(self, msg):
        all_detections = ObjectDetection3DArray()
        self._ego_odom = Odometry()
        for i in range(len(msg.name)):
            if msg.name[i][:len("npc")] == "npc":
                detection = ObjectDetection3D()
                detection.header.stamp = rospy.Time.now()
                detection.header.frame_id = self._namespaces + self._world_frame
                detection.id = 0
                detection.score = 1.0
                detection.state.x = msg.pose[i].position.x
                detection.state.y = msg.pose[i].position.y
                ang = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w])
                detection.state.heading_angle = ang[2]
                detection.state.velocity = math.sqrt(
                    msg.twist[i].linear.x**2 + msg.twist[i].linear.y**2)
                detection.state.acceleration = 0
                detection.bbox.size.x = 0.4
                detection.bbox.size.y = 0.2
                detection.bbox.size.z = 0.3
                detection.bbox.center.position = msg.pose[i].position
                detection.bbox.center.orientation = msg.pose[i].orientation
                all_detections.detections.append(detection)
            if msg.name[i] == self._ego_vehicle_name:
                self._ego_odom.header.stamp = rospy.Time.now()
                self._ego_odom.header.frame_id = self._namespaces + self._world_frame
                self._ego_odom.child_frame_id = self._namespaces + "base_footprint"
                self._ego_odom.pose.pose = msg.pose[i]
                self._ego_odom.twist.twist = msg.twist[i]
                self._ego_odom_pub.publish(self._ego_odom)
        self._object_detection_pub.publish(all_detections)

    def _ego_state_pub_callback(self, event):
        if not self._tf_listener.canTransform(self._namespaces + self._world_frame, self._namespaces + "base_footprint", rospy.Time(0)):
            rospy.logwarn(
                "Cannot get transform from " + self._world_frame + " to base_footprint")
            return
        self._tf_listener.waitForTransform(self._namespaces + self._world_frame,
                                           self._namespaces + "base_footprint", rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self._tf_listener.lookupTransform(self._namespaces +
                                                         self._world_frame, self._namespaces + "base_footprint", rospy.Time(0))
        x, y = trans[0], trans[1]
        euler = tf.transformations.euler_from_quaternion(rot)
        heading_angle = euler[2]
        while (self._ego_odom is None):
            rospy.sleep(0.1)
        ego_state_msg = State()
        ego_state_msg.header.frame_id = self._namespaces + self._world_frame
        ego_state_msg.header.stamp = rospy.Time.now()
        ego_state_msg.name = self._ego_vehicle_name
        ego_state_msg.x = x
        ego_state_msg.y = y
        ego_state_msg.heading_angle = heading_angle
        ego_state_msg.velocity = math.sqrt(
            self._ego_odom.twist.twist.linear.x**2 + self._ego_odom.twist.twist.linear.y**2)
        self._ego_state_pub.publish(ego_state_msg)

    def _ctrl_cmd_callback(self, msg):
        speed = msg.throttle
        steering_angle = msg.steering
        # speed = msg.linear.x
        # steering_angle = msg.angular.z
        
        vel_left_rear_wheel = Float64()
        vel_right_rear_wheel = Float64()
        vel_left_front_wheel = Float64()
        vel_right_front_wheel = Float64()
        pos_left_steering_hinge = Float64()
        pos_right_steering_hinge = Float64()
        wheelbase = 0.26
        thread = 0.2
        wheel_radius = 0.05
        
        if steering_angle != 0:    
            r = wheelbase / math.fabs(math.tan(steering_angle))
            w = speed / r 
            r_left_rear = r - (thread / 2) * math.copysign(1, steering_angle)
            r_right_rear = r + (thread / 2) * math.copysign(1, steering_angle)
            r_left_front = math.sqrt(r_left_rear**2 + wheelbase**2) 
            r_right_front = math.sqrt(r_right_rear**2 + wheelbase**2)
            vel_left_rear_wheel.data = w * r_left_rear / wheel_radius
            vel_right_rear_wheel.data = w * r_right_rear / wheel_radius
            vel_left_front_wheel.data = w * r_left_front / wheel_radius
            vel_right_front_wheel.data = w * r_right_front / wheel_radius
            pos_left_steering_hinge.data = math.atan(wheelbase / r_left_rear) * math.copysign(1, steering_angle)
            pos_right_steering_hinge.data = math.atan(wheelbase / r_right_rear) * math.copysign(1, steering_angle)
            # print("left: ", pos_left_steering_hinge.data)
            # print("right: ", pos_right_steering_hinge.data)
        else:
            vel_left_rear_wheel.data = speed / wheel_radius
            vel_right_rear_wheel.data = speed / wheel_radius
            vel_left_front_wheel.data = speed / wheel_radius
            vel_right_front_wheel.data = speed / wheel_radius
            pos_left_steering_hinge.data = 0
            pos_right_steering_hinge.data = 0
        
        self._pub_vel_left_rear_wheel.publish(vel_left_rear_wheel)
        self._pub_vel_right_rear_wheel.publish(vel_right_rear_wheel)
        self._pub_vel_left_front_wheel.publish(vel_left_front_wheel)
        self._pub_vel_right_front_wheel.publish(vel_right_front_wheel)
        self._pub_pos_left_steering_hinge.publish(pos_left_steering_hinge)
        self._pub_pos_right_steering_hinge.publish(pos_right_steering_hinge)


if __name__ == "__main__":
    rospy.init_node("gazebo_manager_node")
    gazebo_bridge = GazeboBridgeNode()
    rospy.spin()
