# TODO: This should save raw data into a csv
from implementations.agents.reactive_agent import ReactiveAgent
from implementations.state import ActuatorValues


def start_logging(
    log_path="implementations/ai/data/simulation_data.csv", period: float = 10
) -> None:
    with open(log_path, "a") as data_file:
        # log initial column names
        data_file.write(
            '"Time (s)","Temperature 0 (C)","Temperature 1 (C)","Humidity 0","Humidity 1","Light 0","Light 1","Soil Moisture 0","Soil Moisture 1","Weight 0","Weight 1","Fan","Led","Water Pump"'
        )
        data_file.write("\n")

        class LoggingAgent(ReactiveAgent):
            def start_agent(self) -> None:
                import rospy
                from implementations.state import ActuatorValues

                self._parse_arguments()
                self._init_ros()

                rospy.sleep(2)  # Give a chance for the initial sensor values to be read
                while rospy.get_time() == 0:
                    rospy.sleep(0.1)  # Wait for clock to start up correctly

                self.most_recent_log_time = rospy.get_time()  # new line

                # I'm doing this because the setter tries to be smart and update only the different
                self.set_actuator_values(
                    ActuatorValues(False, 0, False), redundant=True
                )

                # think of this as the main loop almost
                while not rospy.core.is_shutdown():
                    self.process_output(rospy.get_time(), "time")

            def process_output(self, data, sensor_type_name) -> None:
                super().process_output(data, sensor_type_name)

                if sensor_type_name == "time":
                    next_time_to_log = self.most_recent_log_time + period

                    if data > next_time_to_log:
                        # log after changing actuator values. this means that the current actuator values will not impact the current sensor data
                        self.randomize_actuators()
                        self.log_current()

                        self.most_recent_log_time = data

            def randomize_actuators(self):
                self.set_actuator_values(ActuatorValues.create_random(), redundant=True)

            def log_current(self):
                data_file.write(str(self.sensor_data.time))
                data_file.write(",")
                data_file.write(str(self.sensor_data.temperature[0]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.temperature[1]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.humidity[0]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.humidity[1]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.light[0]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.light[1]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.soil_moisture[0]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.soil_moisture[1]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.weight[0]))
                data_file.write(",")
                data_file.write(str(self.sensor_data.weight[1]))
                data_file.write(",")
                data_file.write(str(self.actuator_values.fan))
                data_file.write(",")
                data_file.write(str(self.actuator_values.led))
                data_file.write(",")
                data_file.write(str(self.actuator_values.water_pump))
                data_file.write("\n")

        LoggingAgent().start_agent()


print("hi")
if __name__ == "__main__":
    print("we startin")
    # random_value
    start_logging(period=60 * 30)

    print("safely ended")
