from rclpy.action.client import ActionClient
from irobot_create_msgs.action import RotateAngle
from action_handler import ActionHandler

class AngleDriver(ActionHandler):
    def __init__(self, robot_name):
        super().__init__(f"{robot_name}_angle_driver_node")
        self._client = ActionClient(self, RotateAngle, f'/{robot_name}/rotate_angle')

    def _start(self, angle):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        return goal_msg

