from dataclasses import dataclass


@dataclass
class SensorData:
    temperature: tuple[int, int]
    temperature_avg: float
    humidity: tuple[int, int]
    humidity_avg: float
    light: tuple[int, int]
    light_avg: float
    soil_moisture: tuple[int, int]
    soil_moisture_avg: float
    weight: tuple[float, float]
    weight_avg: float
    camera: object  # Ignore for now
    time: object  # TODO


@dataclass
class ActuatorValues:
    fan: bool
    led: int
    water_pump: bool


@dataclass
class State:
    sensor_data: SensorData
    actuator_values: ActuatorValues
