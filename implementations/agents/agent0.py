from single_sensor_data_based_agent import (
    SingleSensorDataBasedAgent,
    SingleSensorDataBasedActuatorFinder,
)
from models.brute_force_best_actuator_finder import GreedyBruteForceBestActuatorFinder
from models.actuator_cost_evaluator import SimpleActuatorCostEvaluator
from models.sensor_data_cost_evaluator import RangeBasedSensorDataCostEvaluator
from models.sensor_data_predictor import SensorDataPredictor


agent0 = SingleSensorDataBasedAgent(
    GreedyBruteForceBestActuatorFinder(
        SimpleActuatorCostEvaluator(RangeBasedSensorDataCostEvaluator(), None)
    )
)


# run the agent
if __name__ == "__main__":
    # the above statement is true even if you run this file with another python script

    agent0.start_agent()
