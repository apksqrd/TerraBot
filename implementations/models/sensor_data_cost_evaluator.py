from state import State, SensorData
from abc import ABC, abstractmethod
from math import sqrt


class SensorDataCostEvaluator(ABC):
    @abstractmethod
    def evaluate_sensor_data_cost(self, sensor_data: SensorData) -> float:
        """Higher cost is worse, 0 is best."""
        raise NotImplementedError


class RangeBasedSensorDataCostEvaluator(SensorDataCostEvaluator):
    def evaluate_sensor_data_cost(self, sensor_data: SensorData) -> float:
        # Sensor Range Limits and Optimal Ranges:
        #  Temperature = [24,29) and [26,27]
        #  Humidity = [70,80) and [75,80]
        #  Light = 8am-10pm [850,950) / 10pm-8am LEDs off and daytime [860,940]
        #  Soil Moisture = [500-650) and [550,600]

        # TODO: time
        return sqrt(
            (sensor_data.temperature_avg - 26.5) ** 2
            + ((sensor_data.humidity_avg - 77.5) / 5) ** 2
            + ((sensor_data.soil_moisture_avg - 600) / 50) ** 2
        )
