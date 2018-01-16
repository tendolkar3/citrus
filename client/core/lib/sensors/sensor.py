from core.lib.physics.utils import constants as c
import math
from core.lib.internal.geometry import perpendicular_distance_of_point_from_line
from core import globals


class GPS:
    @staticmethod
    def get_position(x, y):
        # ToDo: normal distribution around x,y
        return [x, y]


class VehicleSpeedSensor:

    def __init__(self, car=None):
        self.car = car

    def read_speed(self):
        return self.car.speed


class LaneCenterSensor:

    def __init__(self):
        pass

    @staticmethod
    def get_lane_center_dtheta(car):
        lane_center_dtheta = car.current_road.heading - car.heading
        if lane_center_dtheta > math.pi:
            lane_center_dtheta -= 2*math.pi
        return lane_center_dtheta

    def get_lane_center_dy(self, car):

        line = [(car.current_lane.start_x, car.current_lane.start_y),
                (car.current_lane.end_x, car.current_lane.end_y)]
        point = (car.x, car.y)
        lane_center_dy = self.offset_direction(car) * perpendicular_distance_of_point_from_line(point, line_points=line)

        return lane_center_dy

    @staticmethod
    def offset_direction(car):

        current_lane = car.current_road.lanes[car.lane_no]
        end_x, end_y = current_lane.end_x, current_lane.end_y
        road_theta = car.current_road.heading
        car_theta = math.atan2((end_y - car.y), (end_x - car.x))

        if road_theta == math.pi:
            if math.pi - car_theta < road_theta:
                sign = 1
                # lane = self.no_of_lanes // 2 + ind
            elif math.pi - car_theta > road_theta:
                sign = -1
                # lane = self.no_of_lanes // 2 - ind
            else:
                sign = 0
        else:
            if car_theta < road_theta:
                sign = 1
                # lane = self.no_of_lanes // 2 + ind
            elif car_theta > road_theta:
                sign = -1
                # lane = self.no_of_lanes // 2 - ind
            else:
                sign = 0

        return sign


class PositionBasedVelocityEstimator:

    def __init__(self):
        pass

    @staticmethod
    def calc_speed(car):
        if len(car.x_history)<=1:
            return car.speed
        velocity = [
            (car.x_history[-1] - car.x_history[-2])/(1*c.STATE_UPDATE_DT),
            (car.y_history[-1] - car.y_history[-2]) / (1 * c.STATE_UPDATE_DT)
        ]
        return math.sqrt(velocity[0]**2+velocity[1]**2)

    @staticmethod
    def read_speed(speed):
        return speed


class BasicVisual:

    def __init__(self):
        pass

    @staticmethod
    def get_visible_cars(car, front=True, sides=True):
        # ToDo: do not pass car. modify the method
        # ToDo: add get_car_bounding_box methods temporarily
        lane_no = car.get_lane_no()
        visible_cars = dict.fromkeys(['front', 'back', 'left', 'right'], [])
        road = car.get_current_road()

        if front:
            bounding_box = car.get_car_bounding_box_front()
            visible_cars['front'] = road.get_visible_cars_helper(car=car, polygon=bounding_box)
            visible_cars['front'] = road.get_visible_cars_helper(car=car, polygon=bounding_box)

        else:
            visible_cars['front'] = None
        if not sides:
            visible_cars['left'] = None
            visible_cars['right'] = None
        else:
            if lane_no is None:
                print("Error: lane_no is None")
            else:
                if lane_no == 0:
                    bounding_box = car.get_car_bounding_box_adj(road.lane_width, direction='right')
                    visible_cars['right'] = road.get_visible_cars_helper(car=car, polygon=bounding_box)

                elif lane_no == road.no_of_lanes - 1:
                    bounding_box = car.get_car_bounding_box_adj(road.lane_width, direction='left')
                    visible_cars['left'] = road.get_visible_cars_helper(car=car, polygon=bounding_box)
                else:
                    bounding_box = car.get_car_bounding_box_adj(road.lane_width, direction='right')
                    visible_cars['right'] = road.get_visible_cars_helper(car=car, polygon=bounding_box)

                    bounding_box = car.get_car_bounding_box_adj(road.lane_width, direction='left')
                    visible_cars['left'] = road.get_visible_cars_helper(car=car, polygon=bounding_box)

        return visible_cars

    @staticmethod
    def get_traffic_signal_state(car):
        next_node = car.get_next_node()
        traffic_signal = next_node.traffic_signals[car.get_lane_no()]
        return traffic_signal.get_state()


class Polynomial_Filter:
    def __init__(self):
        return

    def filter(self,w,x):
        return sum([wi*xi for wi,xi in zip(w,x)])/(sum(w))