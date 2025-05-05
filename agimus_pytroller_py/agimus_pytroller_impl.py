import importlib

import numpy as np
import pinocchio as pin
from rclpy.serialization import deserialize_message


class ControllerImpl:
    def __init__(self, robot_description: str) -> None:
        pin_model_complete = pin.buildModelFromXML(robot_description)
        locked_joint_names = [
            name
            for name in pin_model_complete.names
            if "finger" in name and name != "universe"
        ]
        locked_joint_ids = [
            pin_model_complete.getJointId(name) for name in locked_joint_names
        ]
        self._pin_model = pin.buildReducedModel(
            pin_model_complete,
            list_of_geom_models=[],
            list_of_joints_to_lock=locked_joint_ids,
            reference_configuration=np.zeros(pin_model_complete.nq),
        )[0]
        self._pin_data = self._pin_model.createData()

        self._p_gains = np.array([60.0, 60.0, 60.0, 60.0, 20.0, 10.0, 5.0])
        self._d_gains = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 1.0])

        self._q_init = np.zeros(self._pin_model.nq)
        self._first_call = True

        self._last_time = None

        self._topic_map = {}

    def build_message_map(self, topic_name, topic_type, callback) -> None:
        if topic_name not in self._topic_map:
            mt = topic_type.split("/")
            messagetype = getattr(importlib.import_module(".".join(mt[:2])), mt[-1])
            self._topic_map[topic_name] = (type(messagetype()), getattr(self, callback))

    def on_message(self, msg_data, topic_name) -> None:
        topic_data = self._topic_map[topic_name]
        # First element in the tuple is message type to deserialize
        msg = deserialize_message(msg_data, topic_data[0])
        # Second element is a function callback
        topic_data[1](msg)

    def dummy_topic_cb(self, msg):
        print(msg, flush=True)

    def jo_mama_cb(self, msg):
        print(msg, flush=True)

    def jo_mama_int_cb(self, msg):
        print(msg, flush=True)

    def on_update(self, state: np.array) -> np.array:
        nq = self._pin_model.nq
        nv = self._pin_model.nv

        if self._first_call:
            self._q_init = state[:nq].copy()
            self._first_call = False

        out = (
            -self._p_gains * (state[:nq] - self._q_init) - self._d_gains * (state[-nv:])
        )

        return out
