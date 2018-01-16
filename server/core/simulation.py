from core.utils.singleton import SingletonDecorator
from core.utils.dot_dict import dotdict
from time import clock, time
from core.globals import init as initialize_map
from core.lib.vehicles.vehicle_factory import VehicleFactory
from random import choice
from heapq import heappush, heappop, nsmallest

config = dotdict({"display_real_time": True,
                  "display_parameters": {
                      "SCREEN_WIDTH": 800,
                      "SCREEN_HEIGHT": 800,
                  },
                  "custom_map": False,
                  "vehicles": {
                      "car": [{
                          "no": 20,
                          "sensors": ["gps"],
                          "physics": [],
                          "driver": "semi",
                          "trip_start_time": 0,
                      }]
                  }
                  }
                 )
# ToDo: get from config file


@SingletonDecorator
class Simulation:
    __real_start_time = 0
    __sim_start_time = 0

    def __init__(self, config):


        # todo : make checks on config
        # multiple cars shouldn't start at the same time at the same location
        if config["display_real_time"]:
            self.SCREEN_WIDTH = config.display_parameters.SCREEN_WIDTH
            self.SCREEN_HEIGHT = config.display_parameters.SCREEN_HEIGHT
            # ToDo: initialize the client for display

        self.clock = clock()
        self.paused = False

        if not config.custom_map:
            initialize_map()

        self.__vehicles = []

        for vehicle_types in config.vehicles.keys():
            for vehicle_type in vehicle_types:
                for _ in vehicle_type.no:
                    v = VehicleFactory.build_vehicle(vehicle_type)
                    self.__vehicles.append((vehicle_type.trip_start_time, v))
                    if "start_node" not in vehicle_type:
                        start_node = choice(map_2d.sources)
                    else:
                        start_node = vehicle_type.start_node
                    if "destination_node" not in vehicle_type:
                        destination_node = choice(map_2d.sinks)
                    else:
                        destination_node = vehicle_type.destination_node

                    VehicleFactory.place_vehicle(v, start_node, destination_node)
                    # fixme: this should be when simulation runs

    def run(self):

        counter = 0
        min_heap = []

        for start_time, v in self.__vehicles:
            heappush(min_heap, (start_time, v))

        self.__real_start_time = time()

        while True:
            sim_time = counter * 0.1
            if nsmallest(1, min_heap)[0] > sim_time:
                _, v = heappop(min_heap)
                v.unlock()

            # update traffic signals
            for intersection in map_2d.intersections_table.values():
                tc = intersection.traffic_controller
                tc.update_signals()

            # update traffic on each road
            for road in map_2d.get_roads():
                road[2]['object'].update_traffic(self.road_net)
                # fixme: update traffic must be modified using the latest data structure
                # features = calculate_traffic_features(road[2]['object'])
                map_2d.update_road_attributes(road[0], road[1], road[2])

            for vehicle in map_2d.get_cars():
                # fixme: update get_cars
                # if car.start_time < (time.time()-simulation_start_time):
                if not vehicle.is_locked():
                    vehicle.update(self.road_net)
                    if vehicle.is_locked():
                        map_2d.remove_car(vehicle)
                        # fixme: add this method to map_2d

            # Todo: write api for communicating with client











