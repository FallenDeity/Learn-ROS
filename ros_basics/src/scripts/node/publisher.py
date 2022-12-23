import typing as t

import genpy
import rospy
from std_msgs import msg

if t.TYPE_CHECKING:
    from rospy.topics import SubscribeListener


__all__: tuple[str, ...] = ("Talker",)


class Talker(rospy.Publisher):
    def __init__(
        self,
        rate: int = 10,
        node_name: str = "Talker",
        *,
        name: str = "Chatter",
        dataclass: t.Type[genpy.Message] = msg.String,
        queue: int = 10,
        listener: t.Optional["SubscribeListener"] = None,
        nodedelay: bool = False,
        latch: bool = False,
        headers: dict[str, t.Any] | None = None,
        auto_establish_node: bool = True,
    ):
        super().__init__(
            name=name,
            data_class=dataclass,
            queue_size=queue,
            subscriber_listener=listener,
            tcp_nodelay=nodedelay,
            latch=latch,
            headers=headers,
        )
        self.node_name = node_name
        self.established = False
        if auto_establish_node:
            self._establish_node()
        self._rate = rospy.Rate(rate) if self.established else None

    def _establish_node(self, *, anonymous: bool = True, **kwargs: t.Any) -> None:
        rospy.init_node(self.node_name, anonymous=anonymous, **kwargs)
        self.established = True

    def send(self, message: t.Any) -> None:
        if isinstance(message, str):
            data = f"{message} ({rospy.get_time()})"
        else:
            data = message
        rospy.loginfo(data)
        self.publish(data)

    def send_messge(self, message: t.Any) -> None:
        if not self.established:
            raise RuntimeError("Node not established.")
        if self._rate is None:
            self.rate = 10
            rospy.logwarn("Setup rate as 10 by default.")
        while not rospy.is_shutdown():
            self.send(message)
            if self._rate is not None:
                self._rate.sleep()

    @property
    def rate(self) -> rospy.Rate:
        return self._rate

    @rate.setter
    def rate(self, rate: int) -> None:
        self._rate = rospy.Rate(rate)


if __name__ == "__main__":
    publisher = Talker(rate=5, dataclass=msg.String)
    try:
        publisher.send_messge(message="Hello World!")
    except rospy.ROSInterruptException:
        ...
