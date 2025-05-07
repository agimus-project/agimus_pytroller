import importlib
from abc import ABC, abstractmethod

import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


class ControllerImpl(ABC):
    @abstractmethod
    def __init__(self, robot_description: str) -> None:
        pass

    def build_message_map(self, topic_name, topic_type, callback) -> None:
        if topic_name not in self._topic_map:
            mt = topic_type.split("/")
            messagetype = getattr(importlib.import_module(".".join(mt[:2])), mt[-1])
            self._topic_map[topic_name] = (type(messagetype()), getattr(self, callback))

    def on_message(self, topic_name: str, msg_data: bytes) -> None:
        topic_data = self._topic_map[topic_name]
        # First element in the tuple is message type to deserialize
        msg = deserialize_message(msg_data, topic_data[0])
        # Second element is a function callback
        topic_data[1](msg)

    def dummy_topic_cb(self, msg: String) -> None:
        pass

    @abstractmethod
    def on_update(self, state: np.array) -> np.array:
        pass
