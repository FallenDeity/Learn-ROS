import enum
import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Path(enum.Enum):
    POSE = "/{name}/pose"
    COMMAND = "/{name}/cmd_vel"
    COLOR = "/{name}/color_sensor"

    @classmethod
    def set_path(cls, name: str, value: "Path") -> str:
        return value.value.format(name=name)


class TurtleSimulator:
    _pose: Pose
    _rate: rospy.Rate

    def __init__(
        self, node_name: str, rate: int, *, name: str = "turtle1", queue: int = 10
    ) -> None:
        self.node_name = node_name
        self._speed = 1
        self._velocity = Twist()
        self._pose_subscriber = rospy.Subscriber(
            Path.set_path(name, Path.POSE), Pose, self._pose_callback
        )
        self._command_publisher = rospy.Publisher(
            Path.set_path(name, Path.COMMAND), Twist, queue_size=queue
        )
        self._establish_node(rate)

    def _establish_node(self, rate: int, *, anonymous: bool = True) -> None:
        rospy.init_node(self.node_name, anonymous=anonymous)
        self._rate = rospy.Rate(rate)

    @staticmethod
    def linear_vel(distance: float, constant: float = 1.5) -> float:
        return distance * constant

    @staticmethod
    def angular_vel(
        steering_angle: float, theta: float, constant: float = 6.0
    ) -> float:
        return constant * (steering_angle - theta)

    @staticmethod
    def steering_angle(x0: float, x1: float, y0: float, y1: float) -> float:
        return math.atan2(y1 - y0, x1 - x0)

    @staticmethod
    def _find_distance(x0: float, x1: float, y0: float, y1: float) -> float:
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def _reset(self) -> None:
        velocity = self._velocity
        velocity.linear.x, velocity.linear.y, velocity.linear.z = 0, 0, 0
        velocity.angular.x, velocity.angular.y, velocity.angular.z = 0, 0, 0

    def _pose_callback(self, pose_: Pose) -> None:
        self._pose = pose_
        self._pose.x = round(self._pose.x, 4)
        self._pose.y = round(self._pose.y, 4)

    def _loop(self, t0: float, duration: float) -> None:
        while rospy.Time.now().to_sec() - t0 < duration:
            self._command_publisher.publish(self._velocity)
            self._rate.sleep()
        self.stop()

    def _move(self, distance: float, reverse: bool = False) -> None:
        self._reset()
        self._velocity.linear.x = self._speed if not reverse else -self._speed
        movement_time = distance / self._speed
        self._loop(rospy.Time.now().to_sec(), movement_time)

    def _rotate(self, angle: float, reverse: bool = False) -> None:
        self._reset()
        rotation_time = round(math.radians(angle) / self._speed, 4)
        self._velocity.angular.z = self._speed if not reverse else -self._speed
        self._loop(rospy.Time.now().to_sec(), rotation_time)

    def move_forward(self, distance: float) -> None:
        self._move(distance)

    def move_backward(self, distance: float) -> None:
        self._move(distance, reverse=True)

    def turn_left(self, angle: float) -> None:
        self._rotate(angle)

    def turn_right(self, angle: float) -> None:
        self._rotate(angle, reverse=True)

    def go_to_goal(self, x: float, y: float, tolerance: float = 0.01) -> None:
        self._reset()
        x_, y_, theta = self._pose.x, self._pose.y, self._pose.theta
        while (d := self._find_distance(x_, x, y_, y)) >= tolerance:
            x_, y_, theta = self._pose.x, self._pose.y, self._pose.theta
            self._velocity.linear.x = self.linear_vel(d)
            self._velocity.angular.z = self.angular_vel(
                self.steering_angle(x_, x, y_, y), theta
            )
            self._command_publisher.publish(self._velocity)
            self._rate.sleep()
        self.stop()

    def stop(self) -> None:
        self._velocity.linear.x = 0
        self._velocity.angular.z = 0
        self._command_publisher.publish(self._velocity)

    @property
    def velocity(self) -> Twist:
        return self._velocity

    @property
    def pose(self) -> Pose:
        return self._pose


if __name__ == "__main__":
    try:
        turtle = TurtleSimulator("turtle_simulator", 10)
        turtle.go_to_goal(1, 1, 0.5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
