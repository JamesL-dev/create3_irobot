import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from irobot_create_msgs.srv import ResetPose
class ResetPoseDriver(Node):
    def __init__(self, robotName):
        super().__init__(f"{robotName}_reset_pose_driver_node")
        self.client = self.create_client(ResetPose, f"{robotName}/reset_pose")

    def start(self):
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        request = ResetPose.Request()
        request.pose = pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
