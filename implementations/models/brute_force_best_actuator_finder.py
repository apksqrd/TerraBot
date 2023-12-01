from implementations.state import ActuatorValues, SensorData
from agents.single_sensor_data_based_agent import SingleSensorDataBasedActuatorFinder
from actuator_cost_evaluator import ActuatorCostEvaluator


class GreedyBruteForceBestActuatorFinder(SingleSensorDataBasedActuatorFinder):
    def __init__(self, actuator_cost_evaluator: ActuatorCostEvaluator) -> None:
        self.actuator_cost_evaluator = actuator_cost_evaluator

    def find_best_actuator_values(
        self, current_sensor_data: SensorData
    ) -> ActuatorValues:
        all_possible_actuator_values = [
            ActuatorValues(fan, led, water_pump)
            for fan in (False, True)
            for led in range(256)
            for water_pump in (False, True)
        ]

        # return the actuator that minimizes cost
        return min(
            all_possible_actuator_values,
            key=lambda possible_actuator_value: self.actuator_cost_evaluator.evaluate_actuator_cost(
                current_sensor_data, possible_actuator_value
            ),
        )
