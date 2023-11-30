from reactive_agent import ReactiveAgent
from abc import ABC, abstractmethod
from typing import Callable
from state import SensorData, State, ActuatorValues


class SingleSensorDataBasedActuatorFinder(ABC):
    @abstractmethod
    def find_best_actuator(self, current_sensor_data: SensorData) -> ActuatorValues:
        raise NotImplementedError


# TODO
class SingleSensorDataBasedAgent(ReactiveAgent):
    def __init__(
        self,
        actuator_finder: SingleSensorDataBasedActuatorFinder,
        agent_name: str = "SingleSensorDataBasedAgent",
    ) -> None:
        super().__init__(agent_name)
        self.actuator_finder = actuator_finder

    def process_output(self, data, sensor_type_name):
        # TODO
        pass
