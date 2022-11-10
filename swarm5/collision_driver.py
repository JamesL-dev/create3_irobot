import rclpy

from angle_driver import AngleDriver
from distance_driver import DistanceDriver

from enum import IntEnum

import math

class CollisionStates(IntEnum):
    JUSTSTARTED=0,
    DIDFIRSTTURN=1,
    DIDFIRSTDRIVE=2,
    DIDSECONDTURN=3,
    DIDSECONDDRIVE=4,
    DIDTHIRDTURN=5,
    ISDONE=6

class CollisionDriver(rclpy.node.Node):
    def __init__(self, robot_name):
        super().__init__(f"{robot_name}_collision_driver_node")
        self.angle_driver = AngleDriver(robot_name)
        self.distance_driver = DistanceDriver(robot_name)
        self.collision_state = CollisionStates.JUSTSTARTED

    def start(self, angleToMove):
        self.collision_state = CollisionStates.JUSTSTARTED
        self.angle_driver.start(angleToMove)

    def stop(self):
        self.angle_driver.stop()
        self.distance_driver.stop()
        self.collision_state = CollisionStates.JUSTSTARTED

    def reset(self, *args, **kwargs):
        self.stop()
        self.start(*args, **kwargs)

    def spin(self):
        doneAngle, _ = self.angle_driver.spin()
        doneDist, _ = self.distance_driver.spin()

        done, accepted = (False, False)

        if self.collision_state == CollisionStates.JUSTSTARTED:
            if doneAngle:
                self.collision_state = CollisionStates.DIDFIRSTTURN
                # drive 0.5m
                self.distance_driver.start(0.5)
                print("Evade!")
            
        elif self.collision_state == CollisionStates.DIDFIRSTTURN:
            if doneDist:
                self.collision_state = CollisionStates.DIDFIRSTDRIVE
                # turn right 90deg
                self.angle_driver.start(-1 * math.pi / 2)
                print("You like That!?")
            
        elif self.collision_state == CollisionStates.DIDFIRSTDRIVE:
            if doneAngle:
                self.collision_state = CollisionStates.DIDSECONDTURN
                # go straight past robot
                self.distance_driver.start(0.5)
                print("Show me your moves!")
            
        elif self.collision_state == CollisionStates.DIDSECONDTURN:
            if doneDist:
                self.collision_state = CollisionStates.DIDSECONDDRIVE
                # turn right 90deg
                self.angle_driver.start(-1 * math.pi / 2)
                print("HAH!")
            
        elif self.collision_state == CollisionStates.DIDSECONDDRIVE:
            if doneAngle:
                self.collision_state = CollisionStates.DIDTHIRDTURN
                self.distance_driver.start(0.5)
                print("Not so fast!")
            
        elif self.collision_state == CollisionStates.DIDTHIRDTURN:
            if doneDist:
                self.collision_state = CollisionStates.ISDONE
                print("Live to die another day")
            
        elif self.collision_state == CollisionStates.ISDONE:
            done = True
        
        return (done, accepted)
