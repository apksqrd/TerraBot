from state import State, SensorData, ActuatorValues
from abc import ABC, abstractmethod
from sensor_data_predictor import SensorDataPredictor
from sensor_data_cost_evaluator import SensorDataCostEvaluator


class ActuatorCostEvaluator(ABC):
    @abstractmethod
    def evaluate_actuator_cost(
        self, current_sensor_data: SensorData, possible_actuator_value: ActuatorValues
    ) -> float:
        raise NotImplementedError


class SimpleActuatorCostEvaluator(ActuatorCostEvaluator):
    def __init__(
        self,
        sensor_data_cost_evaluator: SensorDataCostEvaluator,
        sensor_data_predictor: SensorDataPredictor,
    ) -> None:
        super().__init__()

        self.sensor_data_cost_evaluator = sensor_data_cost_evaluator
        self.sensor_data_predictor = sensor_data_predictor

    def evaluate_actuator_cost(
        self, current_sensor_data: SensorData, possible_actuator_values: ActuatorValues
    ) -> float:
        return self.sensor_data_cost_evaluator.evaluate_sensor_data_cost(
            self.sensor_data_predictor.predict_next_sensor_data(
                State(current_sensor_data, possible_actuator_values)
            )
        )
