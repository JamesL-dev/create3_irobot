import rclpy

from robot import RobotLogic
# Garret address 192.168.0.104

def main(args=None):
    rclpy.init(args=args)
    robot = RobotLogic(robot_id="create3_05AE", mqtt_ip="192.168.0.104")
    robot.start()

if __name__ == "__main__":
    main()
