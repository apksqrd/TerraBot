#!/usr/bin/env python
import argparse
from dataclasses import dataclass
from typing import Tuple
from std_msgs.msg import (
    Float32,
    Int32,
    Int32MultiArray,
    Float32MultiArray,
    Bool,
    String,
)
from topic_def import sensor_types, actuator_types
import rospy


@dataclass
class SensorData:
    time: float = 0
    light_level_avg: float = 0
    moisture_avg: float = 0
    humidity: float = 0
    temperature: float = 0
    weight: float = 0
    water_level: float = 0
    current: float = 0
    energy: float = 0
    light_level_raw: Tuple[float, float] = (0, 0)
    moisture_raw: Tuple[float, float] = (0, 0)
    humidity_raw: Tuple[float, float] = (0, 0)
    temperature_raw: Tuple[float, float] = (0, 0)
    weight_raw: Tuple[float, float] = (0, 0)


class ReactiveAgent:
    def __init__(self, agent_name: str = "Reactive Agent") -> None:
        self.sensor_data: SensorData = SensorData()
        self.agent_name = agent_name

        self.parse_arguments()
        self.init_ros()

    def parse_arguments(self) -> None:
        parser = argparse.ArgumentParser(description=self.agent_name)
        parser.add_argument(
            "-l", "--log", action="store_true", help="print sensor values"
        )
        parser.add_argument(
            "-s", "--sim", action="store_true", help="use simulator")
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
        self.speedup_pub = rospy.Publisher(
            "speedup", Int32, latch=True, queue_size=1)
        self.freq_pub = rospy.Publisher(
            "freq_input", actuator_types["freq"], latch=True, queue_size=1
        )

        rospy.Subscriber(
            "smoist_output", sensor_types["smoist"], self.moisture_reaction, self
        )
        rospy.Subscriber(
            "light_output", sensor_types["light"], self.light_reaction, self
        )
        rospy.Subscriber(
            "level_output", sensor_types["level"], self.level_reaction, self
        )
        rospy.Subscriber(
            "temp_output", sensor_types["temp"], self.temp_reaction, self)
        rospy.Subscriber(
            "humid_output", sensor_types["humid"], self.humid_reaction, self
        )
        rospy.Subscriber(
            "weight_output", sensor_types["weight"], self.weight_reaction, self
        )
        rospy.Subscriber(
            "cur_output", sensor_types["cur"], self.power_reaction, self)

    # def save_moisture(self, data)

    def moisture_reaction(self, data):
        self.sensor_data.moisture_avg = (data.data[0] + data.data[1]) / 2.0
        self.sensor_data.moisture_raw = data.data
        print(type(data))

    def light_reaction(self, data):
        self.sensor_data.light_level_avg = data

    def level_reaction(self):
        pass

    def temp_reaction(self):
        pass

    def humid_reaction(self):
        pass

    def weight_reaction(self):
        pass

    def power_reaction(self):
        pass


print("poop")
# main function
ReactiveAgent()
