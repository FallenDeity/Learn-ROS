import typing as t

import rospy
from rospy_tutorials.srv import AddTwoInts


class Client(rospy.ServiceProxy):
    def __init__(
        self,
        name: str,
        *,
        service_class: t.Any,
        persistent: bool = True,
        headers: dict[str, t.Any] | None = None,
    ) -> None:
        self._init_service(service_name=name, timeout=10)
        super().__init__(
            name=name,
            service_class=service_class,
            persistent=persistent,
            headers=headers,
        )

    def _init_service(self, *, service_name: str, timeout: int = 10) -> None:
        rospy.wait_for_service(service_name, timeout=timeout)
        self.established = True

    def handle(self, *args: t.Any, property_: str, **kwargs: t.Any) -> t.Any:
        try:
            if not self.established:
                raise RuntimeError("Node not established.")
            return getattr(self(*args, **kwargs), property_)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    client = Client("add_two_ints", service_class=AddTwoInts)
    a, b = 5, 7
    print(f"{a} + {b} = {client.handle(a, b, property_='sum')}")
