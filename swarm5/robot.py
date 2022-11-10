from undock_driver import UndockDriver
import rclpy

from odom_sub import OdomSub
from state_setter import StateMachine
from detections_sub import DetectionsSub
from robot_subs import RobotSubs
from goal_driver import GoalDriver
from reset_pose_driver import ResetPoseDriver
from collision_driver import CollisionDriver

from paho.mqtt import client as mqtt
from enum import IntEnum
from geometry_msgs.msg import PoseStamped
import time
import math
from beep_boop import BoopDriver
# import meaning_quotes
# import random

class InternalState(IntEnum):
    DOINGCOLLISION=0,
    ATGOAL=1
    OTHER=2
 
def euler_from_quaternion(orientation):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
            """

        x = orientation['x']
        y = orientation['y']
        z = orientation['z']
        w = orientation['w']

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class RobotLogic():
    def __init__(self, robot_id, mqtt_ip):
        self.mqtt_client = mqtt.Client(f"{robot_id} mqtt client")
        self.mqtt_client.connect(mqtt_ip)
        self.Boopin = BoopDriver(robot_id)
        self.is_doing_collision = False

        # random.seed()

        self.robot_id = robot_id
        self.old_collision = None
        self.odom_sub = OdomSub(robot_id, mqtt_ip)
        self.detections_sub = DetectionsSub(robot_id, mqtt_ip)
        self.all_robots = RobotSubs(["create3_05AE", "dummy"], mqtt_ip)

        self.external_state = StateMachine(robot_id, mqtt_ip)
        self.internal_state = InternalState.OTHER

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0


        self.goal_driver = GoalDriver(robot_id, goal_pose)
        self.collision_driver = CollisionDriver(robot_id)
        self.reset_pose_driver = ResetPoseDriver(robot_id)
        self.undock_driver = UndockDriver(robot_id)

    def isInBounds(self):
        pos = self.all_robots.get_robot(self.robot_id).pose
        return True



    def collisionDetection(self):
        if not self.is_doing_collision: 
            our_robot = self.all_robots.get_robot(self.robot_id)
            if our_robot.pose is None: # we cant check for collision, we dont have our pos
                return
            for robot in self.all_robots.get_robots():
                if robot.id == self.robot_id or robot.pose is None:
                    continue
                r = 0.5
                x,y,z  = euler_from_quaternion(robot.pose['orientation'])
                theta = z
                x = r * math.cos(theta)
                y = r * math.sin(theta)  

                predicted_position_x = our_robot.pose['position']['x'] + x
                predicted_position_y = our_robot.pose['position']['y'] + y

                robot_position = robot.pose["position"]

                if math.dist([predicted_position_x, predicted_position_y], [robot_position['x'], robot_position['y']]) <= 0.3419:
                    print("Collision detected")
                    self.Boopin.startBlocking()
                    self.collision_driver.start(math.pi / 2)
                    self.is_doing_collision = True
                    print(robot.id)
                    return robot.id

            """
            print(robot.id)
            robot_position = robot.pose["position"]
            robot_velocity = magnitude([robot.twist["linear"]["x"],
                                        robot.twist["linear"]["y"],
                                        robot.twist["linear"]["z"]])
            print(robot_position)

            # angle from our position and robot position
            angle = math.atan2(robot_position["y"] - our_robot.pose["position"]["y"],
                               robot_position["x"] - our_robot.pose["position"]["x"])

            # offsetting the angle by the way we are facing
            # angle -= our_robot.pose["rotation"]
            # angle %= math.pi * 2
            # lower_bound = math.pi / 4
            # upper_bound = math.pi * 3 / 4
            # if not (lower_bound <= angle <= upper_bound):
            #     continue
            robot_distance = math.dist((robot_position["x"], robot_position["y"]), 
                         (our_robot.pose["position"]["x"], our_robot.pose["position"]["y"]))
            print(f"robot velocity: {robot_velocity}")
            print(f"distance: {robot_distance}")

            if math.dist((robot_position["x"], robot_position["y"]), 
                         (our_robot.pose["position"]["x"], our_robot.pose["position"]["y"])) <= 0.5 * robot_velocity:
                print("Collision detected")
                self.BoopDriver.startBlocking()
                return robot.id
        return None
            """
    def start(self):
        # undock (blocking)
        self.undock_driver.startBlocking()
        print("Undock complete.")
        # reset pose (blocking)
        self.reset_pose_driver.start()
        print("Pose reset complete.")
        # start moving to goal (nonblocking)
        self.goal_driver.start()
        while rclpy.ok():
            # spinning on mqtt pub/sub nodes
            rclpy.spin_once(self.odom_sub, timeout_sec=0)
            rclpy.spin_once(self.detections_sub, timeout_sec=0)
            rclpy.spin_once(self.external_state, timeout_sec=0)
            rclpy.spin_once(self.all_robots, timeout_sec=0)
            self.external_state.send_state_to_mqtt()
            


            done, accepted = self.goal_driver.spin()
            if done:
                print("Mission Complete")
                self.internal_state = InternalState.ATGOAL
                # print(meaning_quote[random.randint(0,20)])
                # rclpy.shutdown()

            done, _ = self.collision_driver.spin()
            if done: # on end of collision avoidance routine
                print("Collision avoided")
                self.internal_state = InternalState.OTHER
                self.goal_driver.start()
                self.old_collision = None
                self.is_doing_collision = False
                self.collision_driver.stop()
    
            # if there is a collision
            if newCollision := self.collisionDetection() is not None:
                self.goal_driver.stop()
                print("Incomming Robot")
                self.Boopin.startBlocking()
            

                # just started a brand new collision
                if self.internal_state == InternalState.OTHER:
                    # turn left 90deg to start.  
                    self.collision_driver.start(math.pi / 2)
                self.internal_state = InternalState.DOINGCOLLISION

                # if another collision while doing collision
                if newCollision != self.old_collision and self.old_collision is not None:
                    self.old_collision = newCollision
                    self.collision_driver.reset()
            
            time.sleep(0.1)
