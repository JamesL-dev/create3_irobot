from rclpy.action.client import ActionClient
from irobot_create_msgs.action import Undock
from action_handler import ActionHandler

class UndockDriver(ActionHandler):
    def __init__(self, robotName):
        super().__init__(f"{robotName}_undock_driver_node")
        self._client = ActionClient(self, Undock, f'/{robotName}/undock')

    def _start(self):
        goal_msg = Undock.Goal()
        return goal_msg
