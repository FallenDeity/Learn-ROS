import typing as t

import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse

__all__: tuple[str, ...] = ("Server",)
DEFAULT_BUFFER_SIZE = 10


class Server(rospy.Service):
    def __init__(
        self,
        name: str,
        *,
        service_class: t.Any,
        handler: t.Callable[..., t.Any],
        buffer_size: int = DEFAULT_BUFFER_SIZE,
        error_handler: t.Optional[t.Callable[..., t.Any]] = None,
        auto_establish_node: bool = True,
    ) -> None:
        super().__init__(
            name=name,
            service_class=service_class,
            handler=handler,
            buff_size=buffer_size,
            error_handler=error_handler,
        )
        self.node_name = f"{name}_server"
        self.established = False
        if auto_establish_node:
            self._establish_node()

    def _establish_node(self, *, anonymous: bool = True, **kwargs: t.Any) -> None:
        rospy.init_node(self.node_name, anonymous=anonymous, **kwargs)
        self.established = True

    def spin(self) -> None:
        if not self.established:
            raise RuntimeError("Node not established.")
        rospy.spin()


if __name__ == "__main__":
    server = Server(
        "add_two_ints",
        service_class=AddTwoInts,
        handler=lambda req: AddTwoIntsResponse(req.a + req.b),
    )
    server.spin()
