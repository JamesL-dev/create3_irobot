import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from paho.mqtt import client as mqtt
from nav_msgs.msg import Odometry
import json

class OdomSub(Node):
    def __init__(self, robot_id, mqtt_ip):
        super().__init__(f"{robot_id}_odom_sub_node")
        self.mqtt_client = mqtt.Client(f"{robot_id} mqtt odom pub client")
        self.mqtt_client.connect(mqtt_ip)
        self.robot_id = robot_id

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_client = self.create_subscription(
            Odometry,
            "/" + robot_id + "/odom",
            self._result_callback,
            self.qos)


    def _result_callback(self, future):
        pose = future.pose.pose
        twist = future.twist.twist

        json_pose = {
            'pose': {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                },
                'rotation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w,
                }
            },
            'twist': {
                'linear': {
                    'x': twist.linear.x,
                    'y': twist.linear.y,
                    'z': twist.linear.z,
                },
                'angular': {
                    'x': twist.angular.x,
                    'y': twist.angular.y,
                    'z': twist.angular.z
                }
            },
        }

        self.mqtt_client.publish(
            f"irobot_create3_swarm/pose/{self.robot_id}",
            json.dumps(json_pose))
