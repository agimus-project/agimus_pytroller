import importlib
from abc import ABC, abstractmethod
from typing import Any, Callable

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import String


class ControllerImplBase(ABC):
    def __init__(
        self,
        topic_names: list[str],
        topic_types: list[str],
        message_payloads: list[bytes],
    ) -> None:
        self._topic_map: dict[str, tuple[Any, Callable]] = {}
        msg_types = [self.get_topic_type(t) for t in topic_types]
        deserialized_msgs = [
            deserialize_message(data, t) for data, t in zip(message_payloads, msg_types)
        ]
        self.setup(**{name: msg for name, msg in zip(topic_names, deserialized_msgs)})

    @abstractmethod
    def setup(self, **kwargs) -> None:
        """Initialization callback to populate by the user.

        Args:
            kwargs: messages subscribed using ``initialization_data_topics`` ROS parameter
            are passed here as arguments of the function.

        """
        pass

    @staticmethod
    def get_topic_type(topic_type: str) -> Any:
        """Parses message type name, imports it's definition and returns corresponding
        object that can be used to deserialize messages.


        Args:
            topic_type (str): Name of the message type.

        Returns:
            Any: Python object used to store deserialized mesaage.
        """
        mt = topic_type.split("/")
        msg_attr = getattr(importlib.import_module(".".join(mt[:2])), mt[-1])
        return type(msg_attr())

    def build_message_map(
        self, topic_name: str, topic_type: str, callback: str
    ) -> None:
        """Creates a dictionary used as a cache to map topic names with their
        user-class callbacks.

        Args:
            topic_name (str): Name of the topic from which message is subscribed.
            topic_type (str): Name of a type of the topic in a format ``pkg_name/msg/MsgTypeName``.
            callback (str): Name of a python function used as a message callback.
        """
        if topic_name not in self._topic_map:
            message_type = self.get_topic_type(topic_type)
            self._topic_map[topic_name] = (
                message_type,
                getattr(self, callback),
            )

    def on_message(self, topic_name: str, msg_data: bytes) -> None:
        """General callback used by C++ code to pass serialized ROS
        messages, deserialize them in python and then invoke python
        callbacks used as part of end user API.

        Args:
            topic_name (str): Name of the topic from which message is subscribed.
            msg_data (bytes): Serialized message data.
        """
        topic_data = self._topic_map[topic_name]
        # First element in the tuple is message type to deserialize
        msg = deserialize_message(msg_data, topic_data[0])
        # Second element is a function callback
        topic_data[1](msg)

    def _dummy_topic_cb(self, _: String) -> None:
        """Dummy publisher used by the C++ implementation to perform first call of
        the ``on_message`` function. Prevents stall of the real time loop, when
        python compiles it to the bytecode.

        Args:
            _ (String): Deserialized message passed to the callback.
        """
        pass

    def on_publish(self, topic_name: str) -> bytes:
        """Queries for a getter callback for the published message, serializes
        it and returns as bytes to be published from C++ code.

        Args:
            topic_name (str): Name of the topic for the message to be published.

        Returns:
            bytes: Serialized bytes of the published message.
        """
        # Obtain python object for the message
        msg = self._topic_map[topic_name][1]()
        # Serialize python object into message bytes
        data = serialize_message(msg)
        # Pass bytes to C++
        return data

    @abstractmethod
    def on_update(self, state: np.array) -> np.array:
        """Callback called alongside ``update`` function of ros2_control.

        Args:
            state (np.array): State of the robot.

        Returns:
            np.array: Control signal returned from the controller.
        """
        pass

    @abstractmethod
    def on_post_update(self) -> None:
        """Callback called after ``on_update`` finished. Used to perform
        non time-critical actions.
        """
        pass
