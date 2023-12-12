from dataclasses import dataclass, field


@dataclass(frozen=True)
class SensorData:
    temperature: "tuple[int, int]"
    humidity: "tuple[int, int]"
    light: "tuple[int, int]"
    soil_moisture: "tuple[int, int]"
    weight: "tuple[float, float]"
    camera: object  # Ignore for now
    time: float  # in seconds

    temperature_avg: float = field(init=False)
    humidity_avg: float = field(init=False)
    light_avg: float = field(init=False)
    soil_moisture_avg: float = field(init=False)
    weight_avg: float = field(init=False)

    def __post_init__(self):
        temperature_avg = sum(self.temperature) / len(self.temperature)
        humidity_avg = sum(self.humidity) / len(self.humidity)
        light_avg = sum(self.light) / len(self.light)
        soil_moisture_avg = sum(self.soil_moisture) / len(self.soil_moisture)
        weight_avg = sum(self.weight) / len(self.weight)


@dataclass(frozen=True)
class ActuatorValues:
    fan: bool
    led: int
    water_pump: bool


@dataclass(frozen=True)
class State:
    sensor_data: SensorData
    actuator_values: ActuatorValues
