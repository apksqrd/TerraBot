from dataclasses import dataclass


@dataclass
class SensorData:
    temperature: tuple[int, int]
    humidity: tuple[int, int]
    light: tuple[int, int]
    soil_moisture: tuple[int, int]
    weight: tuple[float, float]
    camera: object  # Ignore for now


@dataclass
class ActuatorValues:
    fan: bool
    led: int
    water_pump: bool


@dataclass
class State:
    sensor_data: SensorData
    actuator_values: ActuatorValues


def get_sensor_data_cost(sensor_data: SensorData) -> float:
    # Sensor Range Limits and Optimal Ranges:
    #  Temperature = [24,29) and [26,27]
    #  Humidity = [70,80) and [75,80]
    #  Light = 8am-10pm [850,950) / 10pm-8am LEDs off and daytime [860,940]
    #  Soil Moisture = [500-650) and [550,600]
    # TODO
    return 0
