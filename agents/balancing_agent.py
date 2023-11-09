from reactive_agent import ReactiveAgent

# Sensor Range Limits and Optimal Ranges:
#  Temperature = [24,29) and [26,27]
#  Humidity – [70,80) and [75,80]
#  Light – 8am-10pm [850,950) / 10pm-8am LEDs off and daytime [860,940]
#  Soil Moisture – [500-650) and [550,600]


def clamp(x, minimum, maximum):
    max(min(x, maximum), minimum)


class BalancingAgent(ReactiveAgent):
    def process_output(self, data, sensor_type_name):
        super().process_output(data, sensor_type_name)

        if sensor_type_name == "temp":
            temperature_average = self.sensor_inputs_map["temp_avg"]

            if temperature_average < 26.5:
                # idk how I'm supposed to increase temperature without also changing the light level
                pass

        if sensor_type_name == "humid":
            humidity_average = self.sensor_inputs_map["humid_avg"]

            if humidity_average < 77.5:
                self.wpump_pub.publish(True)

            if humidity_average > 77.5:
                self.wpump_pub.publish(False)

        # if sensor_type_name == "light":
        #     if soil_m
