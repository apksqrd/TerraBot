from state import State, SensorData
from abc import ABC, abstractmethod


class SensorDataPredictor(ABC):
    @abstractmethod
    def predict_next_sensor_data(self, current_state: State) -> SensorData:
        raise NotImplementedError
