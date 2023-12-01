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
# agent0.start_agent()

# TODO Run below code for testing purposes
# print(__name__)
