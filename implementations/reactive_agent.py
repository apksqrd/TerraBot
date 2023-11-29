import argparse
from dataclasses import dataclass
from typing import Callable, Tuple, Dict, Type, Union
from std_msgs.msg import (
    Float32,
    Int32,
    Int32MultiArray,
    Float32MultiArray,
    Bool,
    String,
)
import sys
from topic_def import sensor_types, actuator_types
import rospy
from collections import defaultdict


class ReactiveAgent:
    def __init__(
        self,
        agent_name: str = "ReactiveAgent",
    ) -> None:
        self.sensor_inputs_map: Dict = dict()  # also contains the time
        self.agent_name = agent_name

    def start_agent(self) -> None:
        self.parse_arguments()
        self.init_ros()

        rospy.sleep(2)  # Give a chance for the initial sensor values to be read
        while rospy.get_time() == 0:
            rospy.sleep(0.1)  # Wait for clock to start up correctly

        # think of this as the main loop almost
        while not rospy.core.is_shutdown():
            self.process_output(rospy.get_time(), "time")

    def parse_arguments(self) -> None:
        parser = argparse.ArgumentParser(description=self.agent_name)
        parser.add_argument(
            "-l", "--log", action="store_true", help="print sensor values"
        )
        parser.add_argument("-s", "--sim", action="store_true", help="use simulator")
        args = parser.parse_args()

        self.is_logging = args.log
        self.use_simulator = args.sim

    def init_ros(self):
        if self.use_simulator:
            rospy.set_param("use_sim_time", True)

        rospy.init_node(self.agent_name, anonymous=True)

        self.led_pub = rospy.Publisher(
            "led_input", actuator_types["led"], latch=True, queue_size=1
        )
        self.wpump_pub = rospy.Publisher(
            "wpump_input", actuator_types["wpump"], latch=True, queue_size=1
        )
        self.fan_pub = rospy.Publisher(
            "fan_input", actuator_types["fan"], latch=True, queue_size=1
        )

        self.ping_pub = rospy.Publisher("ping", Bool, latch=True, queue_size=1)

        self.camera_pub = rospy.Publisher(
            "camera", actuator_types["cam"], latch=True, queue_size=1
        )
        self.speedup_pub = rospy.Publisher("speedup", Int32, latch=True, queue_size=1)
        self.freq_pub = rospy.Publisher(
            "freq_input", actuator_types["freq"], latch=True, queue_size=1
        )

        for sensor_type_name in sensor_types.keys():
            rospy.Subscriber(
                sensor_type_name + "_output",
                sensor_types[sensor_type_name],
                self.process_output,
                sensor_type_name,
            )

    def save_output_data(self, data, sensor_type_name):
        self.sensor_inputs_map[sensor_type_name] = data

        if (
            sensor_types[sensor_type_name] == Float32MultiArray
            or sensor_types[sensor_type_name] == Int32MultiArray
        ):
            self.sensor_inputs_map[sensor_type_name + "_avg"] = (data[0] + data[1]) / 2

    # you can and should rewrite this
    def process_output(self, data, sensor_type_name):
        self.save_output_data(data, sensor_type_name)

        if sensor_type_name == "time":
            if not hasattr(self, "last_ping"):
                self.last_ping = 0

            if (self.sensor_inputs_map["time"] - self.last_ping) >= 180:
                self.ping_pub.publish(True)
                self.last_ping = self.sensor_inputs_map["time"]
