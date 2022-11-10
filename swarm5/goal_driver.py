from rclpy.action.client import ActionClient
from irobot_create_msgs.action import NavigateToPosition
from action_handler import ActionHandler

class GoalDriver(ActionHandler):
    # datatype of goal_pose is geometry_msgs/PoseStamped
    def __init__(self, robot_id, goal_pose):
        super().__init__(f"{robot_id}_goal_driver_node")
        self.goal_pose = goal_pose
        self.robot_id = robot_id
        self._client = ActionClient(self, NavigateToPosition, f'/{self.robot_id}/navigate_to_position')

    def _start(self):
        print("Quest Accepted")
        goal_msg = NavigateToPosition.Goal()
        goal_msg.goal_pose = self.goal_pose
        return goal_msg
