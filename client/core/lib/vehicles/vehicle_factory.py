from .car import Car
from .bus import Bus
from .truck import Truck


class VehicleFactory:
    @staticmethod
    def build_vehicle(type):
        # ToDo: pass sensors, driver types as parameters, other configs
        if type == 'car':
            return Car()
        elif type == 'bus':
            return Bus()
        elif type == 'truck':
            return Truck()

    # ToDo: assign route function.

    @staticmethod
    def place_vehicle(vehicle, start_node, end_node, map_2d):

        vehicle.place_on_map(start_node, end_node, map_2d)
        vehicle.lock()


