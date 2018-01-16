from .vehicle import Vehicle
from ..sensors import sensor as sens
from ..physics.utils import constants
from ..physics import update_car_states, fuel_model
# ToDo: update this utils in this folder


class Car(Vehicle):

    def __init__(self, config, map_2d):

        Vehicle.__init__(config.name, config.trip_start_node, config.trip_destination_node, map_2d)

        if config.length:
            self._length = config.length
        else:
            self._length = constants.CAR_LENGTH

        if config.width:
            self._width = config.width
        else:
            self._width = constants.CAR_WIDTH

        self._driver = config.driver

        self._sensor_suite["speed_sensor"] = sens.PositionBasedVelocityEstimator(car=self)
        self._sensor_suite["position_sensor"] = sens.GPS(car=self)
        self._sensor_suite["lane_center_sensor"] = sens.LaneCenterSensor(car=self)

        self._physics["update_state"] = update_car_states
        self._physics["fuel_model"] = fuel_model

    def get_length(self):
        return self._length

    def get_width(self):
        return self._width

