# agimus_pytroller

[![License](https://img.shields.io/badge/License-BSD_2--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/agimus-project/agimus_pytroller/main.svg)](https://results.pre-commit.ci/latest/github/agimus-project/agimus_pytroller/main)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)


> [!WARNING]
> This controller breaks realtime guarantees of ros2_control. Use it at your own risk!

ros2_control controller doubling as a Python abstraction layer for fast prototyping complex control schemes. This controller picks subset of repeating functionalities in designing of robot control strategies and simplifies the development proves.

# Example use

In order to use this implementation two files are required. First one is configuration YAML file that sets up subscribers, publishers and tells the C++ controller to load appropriate Python module and the second one is Python implementation of our controller.

```yaml
my_pytroller:
  ros__parameters:
    type: agimus_pytroller/AgimusPytroller
    # Converted to: from my_amazing_package.my_pytroller import ControllerImpl
    python_module: my_amazing_package.my_pytroller
    # How many control cycles can be skipped. Similar to update_rate, but
    # tailored for this case.
    python_downsample_factor: 2
    # Wether to keep to stall control loop if data is not computed
    # Continues with previous control is no new result arrived
    error_on_no_data: false
    # Perform linear interpolation of control signals
    # during intermediate control cycles (useful in torque control)
    interpolate_trajectory: true
    # Ordered list of input and reference interfaces concatenated
    # into a NumPy array
    input_interfaces:
    - joint1/position
    - filtered_joint2/position
    # Ordered list of command interfaces expected as NumPy array of
    # controller outputs
    command_interfaces:
    - joint1/effort
    - joint2/effort
    # Expected reference interfaces used to be mixed with input interfaces
    reference_interfaces:
    - filtered_joint2/position
    # List of topics waiting on initialization. Passed to `setup` function of
    # controller class with variable names same as values in this list
    initialization_data_topics: [greeting_name]
    greeting_name:
      # Name of the topic
      topic_name: /greeting_name_for_controller
      # Type of a message expected on this topic
      topic_type: std_msgs/msg/String
      # Wether QoS is transient local, useful for `robot_description` topic
      transient_local: true
    # List of topics expected for subscription. Names are only used as labels
    # for parameter matching
    subscribed_topics: [input_angle]
    input_angle:
      topic_name: ~/input_angle
      topic_type: std_msgs/msg/Float
      # Python callback invoked when a message arrived
      python_function_name: input_angle_cb
    # List of topics expected for publishing. Names are only used as labels
    # for parameter matching
    published_topics: [get_greetings]
    get_greetings:
      topic_name: ~/greetings
      topic_type: std_msgs/msg/String
      # Getter invoked at the end of control cycle to obtain populated message
      python_msg_getter_name: get_greetings
```

C++ side of the controller handles creation of subscribers and publishers
making it possible to automatically call appropriate Python functions that have to be defined withing Python controller class. Snippet below explains example toy controller implementing functions defined by the YAML above.


```python
import numpy as np
import numpy.typing as npt

from math import sin

from agimus_pytroller_py.agimus_pytroller_base import ControllerImplBase
from std_msgs.msg import Float, String


class ControllerImpl(ControllerImplBase):
    """Class implementing the controller. There can only be one controller per
    file and it has to be called `ControllerImpl` and inherit from `ControllerImplBase`.
    """

    def __init__(self, *args, **kwargs):
        """Due to serialization/deserialization of initialization messages this
        function has to be implemented with a following body.
        """
        super().__init__(*args, **kwargs)

    def setup(self, greeting_name: String):
        """Function used in the place of `__init__` created to allow passing of
        deserialized messages.

        Args:
            greeting_name (String): Initialization message with a name matching
            the definition from `initialization_data_topics` param.
        """
        self._name = greeting_name.data
        self._target_angles = np.zeros(2)
        self._gains = np.array([1.0, 2.0])
        self._cnt = 0

    def get_greetings(self) -> String:
        """Getter used to obtain message from Python. Serialization is handled
        for the user afterwards. Callback name matches `python_msg_getter_name`
        param.

        Returns:
            String: Serialized message as a Python object.
        """
        return String(data=f"Nice to meet you {self._name}")

    def input_angle_cb(self, msg: Float) -> None:
        """Callback used when new message arrives on a given topic. Name of the
        function has to match `python_function_name` param.

        Args:
            msg (Float): Serialized message as a Python object.
        """
        self._target_angles[0] = msg.data

    def on_update(self, state: npt.ArrayLike) -> npt.ArrayLike:
        """Function invoked every `python_downsample_factor` time of
        `update_and_write_commands()` function calls.

        Args:
            state (npt.ArrayLike): Robot state composed of values obtained from
            `input_interfaces`, with the same order as in the list.

        Returns:
            npt.ArrayLike: Control signal sent to the robot. Values are applied
            in the same order as in `command_interfaces` parameter.
        """
        return (self._target_angles - state) * self._gains

    def on_post_update(self):
        """Optional callback invoked after time-critical section of `on_update`."""
        self._cnt += 1
        self._target_angles[1] = sin(self._cnt / 1000.0)

```

> [!NOTE]
> The fastest this controller can compute control is with a delay of one control cycle! This is an inherit limitation of this architecture.

# Internal architecture

Internally agimus_pytroller spawns a separate thread for Python interpreter to avoid stalling main control loop in cases of Python errors or non-deterministic execution times. The thread starts spinning processing all the incoming messages, passed to it via a queues, invoking appropriate python callbacks with `on_message` python function. The loop is interrupted when `update_and_write_commands` is called in the main thread, passing current state of the robot and invoking `on_update` python function. Once control is computed, results are passed to the main controller thread and `on_publish` is queried to publish messages outside, with `on_post_update` being called afterwards. After that the loop starts processing all remaining messages. See image below for more information on the architecture.

> [!INFO]
> - Rectangles with thick orange line are separate threads
> - Light gray boxes depict functions being called on the python side.

![agimus pytroller architecture](./assets/pytroller_architecture.drawio.svg)


## Acknowledgments

This work is based on an amazing work of [pytroller](https://github.com/ICube-Robotics/pytroller), building on top fo this idea.
