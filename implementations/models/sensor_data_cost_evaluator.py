from state import State, SensorData
from abc import ABC, abstractmethod


class SensorDataCostEvaluator(ABC):
    @abstractmethod
    def evaluate_sensor_data_cost(self, sensor_data: SensorData) -> float:
        """Higher cost is worse, 0 is best."""
        raise NotImplementedError
