from rclpy.action.client import ActionClient
from irobot_create_msgs.action import DriveDistance
from action_handler import ActionHandler

class DistanceDriver(ActionHandler):
    def __init__(self, robot_name):
        super().__init__(f"{robot_name}_distance_driver_node")
        self._client = ActionClient(self, DriveDistance, f'/{robot_name}/drive_distance')

    def _start(self, distance):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        return goal_msg
