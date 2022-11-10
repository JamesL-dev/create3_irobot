import rclpy
from paho.mqtt import client as mqtt
from geometry_msgs.msg import Pose
import geometry_msgs
import array
import time
import json


class Robot:
    def __init__(self, id):
        self.id = id
        self.pose = None
        self.twist = None

class RobotSubs(rclpy.node.Node):
    def __init__(self, robot_ids, mqtt_ip):
        super().__init__(f"create3_05AE_robot_subs_node")

        self.mqtt_client = mqtt.Client(f"create3_05AE mqtt robotsubs client")
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(mqtt_ip)
        self.robot_ids = robot_ids
        self.robot_list = [Robot(id) for id in self.robot_ids]

        for robot_id in self.robot_ids:
            self.mqtt_client.subscribe(
                    f"irobot_create3_swarm/pose/{robot_id}"
                )

        self.mqtt_client.loop_start()

    def on_message(self, client, userdata, msg):
        robot_positions = []
        for robot in self.robot_list:
            if robot.id in msg.topic:
                dictionary = json.loads(msg.payload)
                robot.pose = dictionary['pose']
                robot.twist = dictionary['twist']
                #print(robot.pose)
        #       return
            robot_positions.append(robot)
        self.robot_list = robot_positions

    def get_robots(self):
        return self.robot_list

    def get_robot(self, id):
        for robot in self.robot_list:
            if robot.id == id:
                return robot
        return None

