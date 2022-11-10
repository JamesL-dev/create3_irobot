from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from paho.mqtt import client as mqtt
from irobot_create_msgs.msg import HazardDetectionVector
import json

class DetectionsSub(Node):
    def __init__(self, robot_id, mqtt_ip):
        super().__init__(f"{robot_id}_detections_sub_node")
        self.mqtt_client = mqtt.Client(f"{robot_id} mqtt detections pub client")
        self.mqtt_client.connect(mqtt_ip)
        self.robot_id = robot_id

        # QoS profile:
        #   Reliability: BEST_EFFORT
        #   History (Depth): KEEP_LAST (5)
        #   Durability: VOLATILE
        #   Lifespan: Infinite
        #   Deadline: Infinite
        #   Liveliness: AUTOMATIC
        #   Liveliness lease duration: Infinite
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_client = self.create_subscription(
            HazardDetectionVector,
            "/" + robot_id + "/hazard_detection",
            self._result_callback,
            self.qos)

    def _result_callback(self, future):
        msg = dict()
        msg["detections"] = future.detections
        
        result = self.mqtt_client.publish(
            f"irobot_create3_swarm/robot_hazards/{self.robot_id}",
            str(msg))
