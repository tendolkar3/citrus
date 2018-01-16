from .vehicle import Vehicle
from ..sensors import sensor as sens
from ..physics.utils import constants
# ToDo: update this utils in this folder


class Bus(Vehicle):

    def __init__(self, config, map_2d):

        Vehicle.__init__(config.name, config.trip_start_node, config.trip_destination_node, map_2d)

        if config.length:
            self.__length = config.length
        else:
            self.__length = constants.CAR_LENGTH

        if config.width:
            self.__width = config.width
        else:
            self.__width = constants.CAR_WIDTH

        self._driver = config.driver

        self._sensors = dict()
        self._sensors["speed_sensor"] = sens.PositionBasedVelocityEstimator(car=self)
        self._sensors["position_sensor"] = sens.GPS(car=self)
        self._sensors["lane_center_sensor"] = sens.LaneCenterSensor(car=self)

