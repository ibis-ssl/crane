import math
import time

import rclpy
from rclpy.node import Node
from robocup_ssl_msgs.msg import RobotId, TrackedFrame

from crane_msgs.msg import RobotCommands


class MeasureLatency(Node):
    def __init__(self):
        super().__init__("measure_latency")
        self.detection_sub = self.create_subscription(
            TrackedFrame, "/detection_tracked", self.detection_callback, 10
        )
        self.command_sub = self.create_subscription(
            RobotCommands, "/robot_commands", self.command_callback, 10
        )
        self.robot_id = 0
        self.move_start_time = None
        self.pre_vel = 0.0
        self.VEL_THRESHOLD = 0.1
        # self.is_waiting_for_moving = False

    def detection_callback(self, msg: TrackedFrame):
        # print("detection_callback")
        if self.move_start_time is not None:
            robot = None
            for r in msg.robots:
                if r.robot_id.id == self.robot_id and r.robot_id.team == RobotId.TEAM_COLOR_BLUE:
                    if len(r.vel) > 0:
                        vel_2d = r.vel[0]
                        vel = math.sqrt(vel_2d.x**2 + vel_2d.y**2)
                        if vel > self.VEL_THRESHOLD:
                            self.get_logger().info(
                                "start detected! latency: {}".format(
                                    (time.time() - self.move_start_time) * 1000
                                )
                            )
                            self.move_start_time = None
                            break

    def command_callback(self, msg):
        # print("command_callback")
        robot = None
        for command in msg.robot_commands:
            if command.robot_id == self.robot_id:
                robot = command
                break
        if robot is None:
            return
        # get target velocity size
        vel = math.sqrt(robot.target_velocity.x**2 + robot.target_velocity.y**2)
        # print(vel)
        if (
            self.pre_vel < self.VEL_THRESHOLD
            and vel > self.VEL_THRESHOLD
            and self.move_start_time is None
        ):
            print("start moving")
            self.move_start_time = time.time()
        self.pre_vel = vel


if __name__ == "__main__":
    rclpy.init()
    node = MeasureLatency()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
