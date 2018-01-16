# from ..physics import
from time import time
from ..sensors.sensor_factory import SensorFactory
from math import sqrt


class Vehicle:

    def __init__(self, name, sensors, physics, driver):
        # ToDo: start_node destination_node are not required.
        # ToDo: Take x,y as params

        self._ID = str(hash(time()))+"_"+name
        self._name = name
        # ToDo: Add a physics module (engine, front wheel drive, etc)
        self._physics = dict.fromkeys(seq=["update_state", "fuel_model"], value=None)

        self._sensor_suite = dict()
        for sensor in sensors:
            self._sensor_suite[sensor] = SensorFactory.build_sensor(sensor)

        self._engine_model = None
        self._driver_model = driver
        self._internal_car_communication = None
        self._v2x = None
        # self._ECU = None

        self._perceived_attr = dict.fromkeys(seq=["visible_cars","gps","speed","traffic_state"])
        self._localized_attr = dict.fromkeys(
            seq=["lane_dtheta","lane_dy","distance_from_front_car","distance_from_back_car","distance_from_next_node"])

        self._perceived_attr["visible_cars"] = dict.fromkeys(seq=["front", "back", "left", "right"], value=[])
        self._perceived_attr["gps"] = [None, None]
        self._perceived_attr["speed"] = None
        self._perceived_attr["traffic_state"] = None
        # ToDo: determine a traffic_state proto that will be updated for every car.

        self._localized_attr["lane_dtheta"] = None
        self._localized_attr["lane_dy"] = None
        self._localized_attr["distance_from_front_car"] = None
        self._localized_attr["distance_from_back_car"] = None
        self._localized_attr["distance_from_next_node"] = None

        self._x = None
        self._y = None
        self._length = 0
        self._width = 0
        self._color = None
        self._lock = True
        # ToDo: map_2d should have a sources attribute (parking lots). And choose random node as start_node as default
        self._lane_center_dy = None
        self._lane_center_dtheta = None
        self._speed = 0
        self._acceleration = 0
        self._heading = None
        self._lane_change_flag = False
        self._route = None
        self._history = list()

        self._trip_start_time = 0
        self._trip_destination_node = None
        self._trip_start_node = None
        self._current_node = self._trip_start_node
        self._next_node = None
        self._current_road = None
        self._lane_no = None
        # self._current_road = map_2d.get_road((self._current_node.id, self._next_node.id))
        # self._lane_no = self._current_road.get_lane(self._x, self._y)

        # ToDo: Add a lane_check and road_check to check if the vehicle is actually in the intended place
        # future :  Perform consistency checks

    def place_on_map(self, trip_start_node, trip_destination_node, map_2d):
        """
        It assigns a trip start node and destination node. Also it places the vehicle on the map.
        :param trip_start_node:
        :param trip_destination_node:
        :param map_2d:
        :return:
        """
        self._current_node = self._trip_start_node
        self._route = self.calculate_route(trip_start_node, trip_destination_node, map_2d)
        self._next_node = self._route[1]

        self._x = map_2d[trip_start_node].X if trip_start_node is not None else map_2d[1].X
        self._y = map_2d[trip_start_node].Y if trip_start_node is not None else map_2d[1].Y
        self._current_road = map_2d.get_road((self._current_node.id, self._next_node.id))
        self._heading = self._current_road.heading
        self._lane_change_flag = False
        self._route = None
        self._history = list()

        self._trip_destination_node = trip_destination_node
        self._trip_start_node = trip_start_node
        self._current_node = self._trip_start_node
        self._next_node = None
        self._lane_no = self._current_road.get_lane(self._x, self._y)

        self._current_road.lanes[self._lane_no].insert_vehicle(self)
        # fixme: add a method on lane object to insert vehicles in order of their distance from next node

    def calculate_route(self, trip_start_node, trip_destination_node, map_2d):
        # fixme: add routing
        return

    def get_id(self):
        return self._ID

    def get_name(self):
        return self._name

    def set_name(self, name):
        self._name = name

    def get_x(self):
        return self._x

    def set_x(self, x):
        self._x = x

    def get_y(self):
        return self._y

    def set_y(self, y):
        self._y = y

    def get_lane_center_dy(self):
        return self._lane_center_dy

    def set_lane_center_dy(self, dy):
        self._lane_center_dy = dy

    def get_lane_center_dtheta(self):
        return self._lane_center_dtheta

    def set_lane_center_dtheta(self, dtheta):
        self._lane_center_dtheta = dtheta

    def get_speed(self):
        return self._speed

    def set_speed(self, speed):
        self._speed = speed

    def get_acceleration(self):
        return self._acceleration

    def set_acceleration(self, acceleration):
        self._acceleration = acceleration

    def get_heading(self):
        return self._heading

    def set_heading(self, heading):
        self._heading = heading

    def get_lane_change_flag(self):
        return self._lane_change_flag

    def set_lane_change_flag(self, flag):
        self._lane_change_flag = flag

    def get_lane_no(self):
        return self._lane_no

    def set_lane_no(self, lane_no):
        self._lane_no = lane_no

    def get_current_node(self):
        return self._current_node

    def set_current_node(self, node):
        self._current_node = node

    def get_next_node(self):
        return self._next_node

    def set_next_node(self, node):
        self._next_node = node

    def get_current_road(self):
        return self._current_road

    def update_current_road(self, map_2d):
        self._current_road = map_2d.get_road((self._current_node.id, self._next_node.id))

    def get_route(self):
        return self._route

    def set_route(self, route):
        self._route = route

    def get_history(self):
        return self._history

    def update_history(self, reading):
        self._history.append(reading)

    def lock(self):
        self._lock = True

    def unlock(self):
        self._lock = False

    def move(self, steering_angle, gas, brake):

        self._x, self._y, self._heading, self._speed = self._physics["update_state"].two_wheel_drive(
            x=self._x, y=self._y, heading=self._heading, speed=self._speed, length=self._length,
            steering_angle=steering_angle, gas=gas, brake=brake, gas_to_acc=1, brake_to_acc=1)
        print("x:", self._x, "Y:", self._y, "heading:", self._heading, "speed:", self._speed)

    def perceive(self):

        for key in self._perceived_attr.keys():
            if key == "visible_cars":
                self._perceived_attr[key] = self._sensor_suite["basic_visual"].get_visible_cars()

            elif key == "gps":
                self._perceived_attr[key] = self._sensor_suite["gps"].get_position(self._x, self._y)

            elif key == "speed":
                self._perceived_attr[key] = self._sensor_suite["speed"].read_speed(self._speed)

            elif key == "traffic_signal_state":
                self._perceived_attr[key] = self._sensor_suite["basic_visual"].get_traffic_signal_state(self)

            else:
                pass

    def localize(self):

        for key in self._localized_attr.keys():
            if key == "lane_dtheta":
                self._localized_attr[key] = self._sensor_suite["lane_center"].get_lane_center_dtheta(self)

            elif key == "lane_dy":
                self._localized_attr[key] = self._sensor_suite["lane_center"].get_lane_center_dy(self)

            elif key == "distance_from_front_car":
                # returns the front car and the distance
                front_cars = self._perceived_attr["visible_cars"]["front"]
                return min([sqrt((c.get_y() - self._y) ** 2 + (c.get_x() - self._x) ** 2) for c in front_cars]) if len(
                    front_cars) > 0 else c.CAR_SAFE_DIST + 10

            elif key == "distance_from_back_car":
                back_cars = self._perceived_attr["visible_cars"]["back"]
                return min([sqrt((c.get_y() - self._y) ** 2 + (c.get_x() - self._x) ** 2) for c in back_cars]) if len(
                    back_cars) > 0 else c.CAR_SAFE_DIST + 10

            elif key == "distance_from_next_node":
                next_node = self.get_next_node()
                return sqrt((self._x - next_node.X)**2 + (self._y - next_node.Y)**2)

            else:
                pass

    def driving_decision(self, reroute_flag=False):

        return 0, 0, 0

    def step(self):

        self.perceive()
        """
        should update the sensor readings like velocity, front cars, back cars, traffic signals etc
        """

        self.localize()
        """
        # infer from the sensor readings, i.e. distance from font car, lane center dy etc
        """

        steering_angle, gas, brake = self.driving_decision(reroute_flag=False)
        """
        should include a flag which determines if it has to be rerouted
        - includes routing which is done at a separate frequency
        - must call driver module
        - 
        """

        self.move(steering_angle, gas, brake)
