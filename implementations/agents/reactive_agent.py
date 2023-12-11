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
from implementations.state import SensorData, State, ActuatorValues


class ReactiveAgent:
    def __init__(
        self,
        agent_name: str = "ReactiveAgent",
    ) -> None:
        self.__sensor_data: SensorData = SensorData(
            (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), None, 0
        )
        self.__actuator_values: ActuatorValues = ActuatorValues(False, 0, False)
        self.agent_name = agent_name

    def start_agent(self) -> None:
        self.__parse_arguments()
        self.__init_ros()

        rospy.sleep(2)  # Give a chance for the initial sensor values to be read
        while rospy.get_time() == 0:
            rospy.sleep(0.1)  # Wait for clock to start up correctly

        # think of this as the main loop almost
        while not rospy.core.is_shutdown():
            self.process_output(rospy.get_time(), "time")

        # I'm doing this because the setter tries to be smart and update only the different
        self.set_actuator_values(ActuatorValues(False, 0, False), redundant=True)

    def __parse_arguments(self) -> None:
        parser = argparse.ArgumentParser(description=self.agent_name)
        parser.add_argument(
            "-l", "--log", action="store_true", help="print sensor values"
        )
        parser.add_argument("-s", "--sim", action="store_true", help="use simulator")
        args = parser.parse_args()

        self.is_logging = args.log
        self.use_simulator = args.sim

    def __init_ros(self):
        if self.use_simulator:
            rospy.set_param("use_sim_time", True)

        rospy.init_node(self.agent_name, anonymous=True)

        self.__led_publisher = rospy.Publisher(
            "led_input", actuator_types["led"], latch=True, queue_size=1
        )
        self.__water_pump_publisher = rospy.Publisher(
            "wpump_input", actuator_types["wpump"], latch=True, queue_size=1
        )
        self.__fan_publisher = rospy.Publisher(
            "fan_input", actuator_types["fan"], latch=True, queue_size=1
        )
        self.__ping_publisher = rospy.Publisher("ping", Bool, latch=True, queue_size=1)
        self.__camera_publisher = rospy.Publisher(
            "camera", actuator_types["cam"], latch=True, queue_size=1
        )
        self.__freq_publisher = rospy.Publisher(
            "freq_input", actuator_types["freq"], latch=True, queue_size=1
        )

        # for simulations
        self.__speedup_publisher = rospy.Publisher(
            "speedup", Int32, latch=True, queue_size=1
        )

        for sensor_type_name in sensor_types.keys():
            rospy.Subscriber(
                sensor_type_name + "_output",
                sensor_types[sensor_type_name],
                self.process_output,
                sensor_type_name,
            )

    def __save_output_data(self, data, sensor_type_name: String):
        if sensor_type_name == "weight":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                self.sensor_data.soil_moisture,
                data,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )
        if sensor_type_name == "smoist":
            self.__sensor_data = SensorData(
                self.sensor_data.temperature,
                self.sensor_data.humidity,
                self.sensor_data.light,
                data,
                self.sensor_data.weight,
                self.sensor_data.camera,
                self.sensor_data.time,
            )

    # you can and should rewrite this
    def process_output(self, data, sensor_type_name):
        self.__save_output_data(data, sensor_type_name)

        if sensor_type_name == "time":
            if not hasattr(self, "last_ping"):
                self.__last_ping = 0

            if (self.__sensor_data.time - self.__last_ping) >= 180:
                self.__ping_publisher.publish(True)
                self.__last_ping = self.__sensor_data.time

    @property
    def sensor_data(self) -> SensorData:
        return self.__sensor_data

    @property
    def actuator_values(self) -> ActuatorValues:
        return self.__actuator_values

    @property
    def state(self) -> State:
        return State(self.sensor_data, self.actuator_values)

    def set_actuator_values(self, actuator_values: ActuatorValues, redundant: Bool):
        # I didn't annotate this with @actuator_values.setter because it didn't support optional arguments

        if redundant:
            # update all actuators, even if they are the same
            self.__fan_publisher.publish(actuator_values.fan)
            self.__led_publisher.publish(actuator_values.led)
            self.__water_pump_publisher.publish(actuator_values.water_pump)
        else:
            # update only the different actuator values
            if actuator_values.fan != self.actuator_values.fan:
                self.__fan_publisher.publish(actuator_values.fan)
            if actuator_values.led != self.actuator_values.led:
                self.__led_publisher.publish(actuator_values.led)
            if actuator_values.water_pump != self.actuator_values.water_pump:
                self.__water_pump_publisher.publish(actuator_values.water_pump)

        self.__actuator_values = actuator_values
