import typing as t

import genpy
import rospy
from std_msgs import msg

DEFAULT_BUFF_SIZE = 65536

__all__: tuple[str, ...] = ("Listener",)


class Listener(rospy.Subscriber):
    def __init__(
        self,
        node_name: str = "Listener",
        *,
        name: str = "Chatter",
        dataclass: t.Type[genpy.Message] = msg.String,
        callback: t.Callable[[t.Any], t.Any] = lambda d, *_: rospy.loginfo(
            f"{rospy.get_caller_id()} I heard {d.data}!"
        ),
        callback_args: tuple[t.Any, ...] | None = None,
        queue_size: int | None = None,
        buff_size: int = DEFAULT_BUFF_SIZE,
        tcp_nodelay: bool = False,
        auto_establish_node: bool = True,
    ) -> None:
        super().__init__(
            name=name,
            data_class=dataclass,
            callback=callback,
            callback_args=callback_args,
            queue_size=queue_size,
            buff_size=buff_size,
            tcp_nodelay=tcp_nodelay,
        )
        self.node_name = node_name
        self.establised = False
        if auto_establish_node:
            self._establish_node()

    def _establish_node(self, *, anonymous: bool = True, **kwargs: t.Any) -> None:
        rospy.init_node(self.node_name, anonymous=anonymous, **kwargs)
        self.established = True

    @staticmethod
    def listen() -> None:
        rospy.spin()

    @classmethod
    def listen_deco(
        cls, *args: t.Any, **kwargs: t.Any
    ) -> t.Callable[[t.Callable], "Listener"]:
        def wrapper(func: t.Callable) -> "Listener":
            nonlocal args, kwargs
            self = cls(*args, **kwargs, callback=func)
            self.listen()
            return self

        return wrapper


if __name__ == "__main__":
    Listener(dataclass=msg.Int16).listen()

    # @Listener.listen_deco(callback_args=("Hello", "World", "!"))
    # def reply(data: String, *args: tuple[t.Any, ...]) -> None:
    #     rospy.loginfo(f"{', '.join(args[0])} {data.data}")
