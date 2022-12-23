import typing as t

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


__all__: tuple[str, ...] = ("Turtle",)


class Turtle:
    def __init__(self, name: str, *, rate: int = 10000) -> None:
        self.name = name
        self.rate = rate
        self._pose: t.Optional[Pose] = None
        self._velocity: t.Optional[Twist] = None
        self._pose_subscriber = rospy.Subscriber(
            f"/{self.name}/pose", Pose, self._pose_callback
        )
        self._velocity_publisher = rospy.Publisher(
            f"/{self.name}/cmd_vel", Twist, queue_size=10
        )
        self._establish_node()

    def _establish_node(self, *, anonymous: bool = True, **kwargs: t.Any) -> None:
        rospy.init_node(self.name, anonymous=anonymous, **kwargs)
        self._rate = rospy.Rate(self.rate)

    def _pose_callback(self, data: Pose) -> None:
        self._pose = data

    def _set_velocity(self, velocity: Twist) -> None:
        self._velocity = velocity

    def _move(self, velocity: Twist) -> None:
        self._set_velocity(velocity)
        self._velocity_publisher.publish(self._velocity)

    def move(self, velocity: Twist) -> None:
        self._move(velocity)

    def move_forward(self, speed: float = 2.0) -> None:
        velocity = Twist()
        velocity.linear.x = speed
        self.move(velocity)

    def move_backward(self, speed: float = 2.0) -> None:
        velocity = Twist()
        velocity.linear.x = -speed
        self.move(velocity)

    def move_left(self, speed: float = 2.0) -> None:
        velocity = Twist()
        velocity.linear.y = speed
        self.move(velocity)

    def move_right(self, speed: float = 2.0) -> None:
        velocity = Twist()
        velocity.linear.y = -speed
        self.move(velocity)

    def rotate(self, velocity: Twist) -> None:
        self._move(velocity)

    def rotate_clockwise(self, speed: float = 2.0) -> None:
        velocity = Twist()
        velocity.angular.z = -speed
        self.rotate(velocity)

    def rotate_anticlockwise(self, speed: float = 2.0) -> None:
        velocity = Twist()
        velocity.angular.z = speed
        self.rotate(velocity)

    def stop(self) -> None:
        velocity = Twist()
        self._move(velocity)

    def get_pose(self) -> Pose:
        return self._pose


if __name__ == "__main__":
    turtle = Turtle("turtle1", rate=200)
    radius = 1
    while not rospy.is_shutdown():
        x = 1 / 360 * 2 * 3.14 * radius
        turtle.move_forward(x)
        turtle.rotate_clockwise(1 / 360 * 2 * 3.14)
