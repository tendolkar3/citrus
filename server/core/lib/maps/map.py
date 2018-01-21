from rtree import index
import networkx as nx
import itertools
import math
import numpy as np
# from core.lib.maps.utils import constants as c
from shapely.geometry import Polygon, LineString, Point
from core.lib.infrastructure.traffic import TrafficController
from core.helpers import norm, perpendicular_distance_of_point_from_line
from core.lib.maps.generate_traffic_graph import create_sample_network


class SubNode:
    def __init__(self, point_x, point_y, grade=0):
        self.__x = point_x
        self.__y = point_y
        self.__grade = grade

    def get_x(self):
        return self.__x

    def set_x(self, x):
        self.__x = x

    def get_y(self):
        return self.__y

    def set_y(self, y):
        self.__y = y

    def get_grade(self):
        return self.__grade

    def set_grade(self, grade):
        self.__grade = grade


class RoadString:

    def __init__(self, line_string):
        self.__sub_nodes = None
        self.__line_string = line_string
        # An osm road has a line string of length n. sub_nodes will be of length n-1 (excluding the first node).
        self.__perpendicular_segments = None
        # n-1 segments
        # not actually perpendicular though
        self.__segment_headings = None
        # n-1 segment headings
        self.__no_sub_nodes = 0
        self.__length_segments = None
        self.set_attributes(line_string)

    def get_line_string(self):
        return self.__line_string

    def set_attributes(self, line_string):
        x, y = line_string.coords.xy
        # print(line_string.coords.xy)
        self.__sub_nodes = [SubNode(x[i], y[i]) for i in range(1, len(x))]
        self.__segment_headings = [math.atan2(y[i]-y[i-1], x[i]-x[i-1]) for i in range(1, len(x))]
        # print(len(self.__segment_headings), list(range(1, len(x)-1)))
        self.__perpendicular_segments = [self.get_perpendicular_segment(
            self.__segment_headings[i], self.__segment_headings[i+1], (x[i], y[i]))
            for i in range(len(self.__segment_headings)-1)]
        self.__perpendicular_segments.append(self.get_perpendicular_segment(
            self.__segment_headings[len(x)-2], self.__segment_headings[len(x)-2], (x[-1], y[-1])))
        self.__no_sub_nodes = len(self.__sub_nodes)
        self.__length_segments = [norm((x[i]-x[i-1], y[i]-y[i-1])) for i in range(1, len(x))]

    @staticmethod
    def get_perpendicular_segment(theta1, theta2, point):
        x, y = point
        slope = math.tan((theta1 + theta2) / 2)
        c = y - slope * x
        return 1, -slope, -c

    def get_subnodes(self):
        return self.__sub_nodes

    def get_perpendicular_segments(self):
        return self.__perpendicular_segments

    def get_no_sub_nodes(self):
        return self.__no_sub_nodes

    def get_segment_headings(self):
        return self.__segment_headings

    def get_segment_lengths(self):
        return self.__length_segments


class Node:
    def __init__(self, point_x, point_y, incoming, grade=0, _id=None, parent=None):
        """

        :param point_x:
        :param point_y:
        :param incoming: True or False
        :param grade:
        :param _id:
        :param parent:
        """
        self.__x = point_x
        self.__y = point_y
        self.__grade = grade
        self.__parent = parent
        self.__id = _id
        self.__traffic_signals = None
        self.__incoming = incoming
        # self._traffic_signals = dict().fromkeys(range(3))

    def get_x(self):
        return self.__x

    def set_x(self, x):
        self.__x = x

    def get_y(self):
        return self.__y

    def set_y(self, y):
        self.__y = y

    def get_grade(self):
        return self.__grade

    def set_grade(self, grade):
        self.__grade = grade

    def get_id(self):
        return self.__id

    def set_id(self, _id):
        self.__id = _id

    def is_incoming(self):
        return self.__incoming


class Intersection:
    def __init__(self, point_x, point_y, grade=0, osmid=None, traffic_controller=None):
        self.__x = point_x
        self.__y = point_y
        self.__grade = grade
        self.__osmid = osmid
        self.__type = None
        self.__nodes = []
        self.__traffic_controller = traffic_controller
        self.__no_of_nodes = 0

    def get_x(self):
        return self.__x

    def set_x(self, x):
        self.__x = x

    def get_y(self):
        return self.__y

    def set_y(self, y):
        self.__y = y

    def get_grade(self):
        return self.__grade

    def set_grade(self, grade):
        self.__grade = grade

    def get_osmid(self):
        return self.__osmid

    def set_osmid(self, osmid):
        self.__osmid = osmid

    def get_nodes(self):
        return self.__nodes

    def add_node(self, node):
        self.__nodes.append(node)
        self.__no_of_nodes += 1

    def get_traffic_controller(self):
        return self.__traffic_controller

    def get_no_of_nodes(self):
        return self.__no_of_nodes


class Lane:
    """
    using the data structure to store the traffic in order of distance from end node
    """
    def __init__(self, lane_no, road, _type=None):

        self.__vehicles = []
        self.__lane_no = lane_no
        self.__lane_type = _type
        self.__road = road

    def insert_vehicle(self, vehicle):

        # todo: remove the dependency on vehicle methods. Should be able to calculate it from vehicle x,y
        """
        This method has to be on car. Just use the distance from road_end as an attribute on car. Use that to sort
        seg_lengths = road_string.get_segment_lengths()
        next_perpendicular_segment = road_string.get_perpendicular_segments()[segment_no]
        a, b, c = next_perpendicular_segment
        x, y = vehicle.get_x(), vehicle.get_y()
        distance = sum(seg_lengths[segment_no+1:]) + abs(a*y + b*x + c)/math.sqrt(a**2 + b**2)
        """
        distance = vehicle.get_distance_from_road_end()
        for ind, v in enumerate(self.__vehicles):

            if distance > v.get_distance_from_road_end():
                pass
            else:
                self.__vehicles.insert(ind, vehicle)
                break

    def remove_vehicle(self, vehicle):

        try:
            self.__vehicles.remove(vehicle)
        except ValueError:
            print("vehicle not found in lane.__vehicles")


class Road:
    def __init__(self, start_node, end_node, road_string, _type='straight', no_of_lanes=3, name=None,
                 osmid=None, lane_width=4, speed_limit=50):
        self.__start_node = start_node
        self.__end_node = end_node
        self.__type = _type
        self.__speed_limit = speed_limit
        self.__road_string = road_string
        self.__lanes = []
        self.__length = sum(self.__road_string.get_segment_lengths())
        self.__name = name
        self.__osmid = osmid
        self.__no_of_lanes = no_of_lanes
        self.__lane_width = lane_width
        self.__lanes = [Lane(lane_no=i, road=self) for i in range(no_of_lanes)]

    def get_name(self):
        return self.__name

    def get_road_length(self):
        return self.__length

    def get_road_string(self):
        return self.__road_string

    def get_lane(self, x, y):
        # adding some epsilon value
        distance = self.get_distance_from_road_center(x, y)
        road_theta = self.heading
        car_theta = math.atan2((self.end_node.Y - y), (self.end_node.X - x))
        lane = None
        if not (self.bounding_polygon.touches(Point(x, y)) or self.bounding_polygon.contains(Point(x, y))):

            lane = None  # implies that the point is not on the given road
            # print('here 1')
        elif self.no_of_lanes % 2 == 1:
            # print('here2')
            for ind in range(1 + self.no_of_lanes//2):
                if distance < (self.lane_width/2 + self.lane_width * ind):
                    if ind == 0:
                        lane = self.no_of_lanes//2
                    else:
                        if road_theta == math.pi:
                            if math.pi - car_theta < road_theta:
                                lane = self.no_of_lanes//2 + ind
                            else:
                                lane = self.no_of_lanes//2 - ind
                        else:
                            if car_theta < road_theta:
                                lane = self.no_of_lanes//2 + ind
                            else:
                                lane = self.no_of_lanes//2 - ind
                    # print("lane : ", lane)
                    break

        else:
            # print('here3')
            for ind in range(1, 1+self.no_of_lanes//2):
                if distance < self.lane_width * ind:
                    if road_theta == math.pi:
                        if math.pi - car_theta < road_theta:
                            lane = self.no_of_lanes//2 + ind - 1
                        else:  # inner edge of the road
                            lane = self.no_of_lanes//2 - ind
                        break
                    else:
                        if car_theta < road_theta:              # outer edge of the road
                            lane = self.no_of_lanes//2 + ind - 1
                        else:                                   # inner edge of the road
                            lane = self.no_of_lanes//2 - ind
                        break

        return lane

    def get_distance_from_road_center(self, x, y):
        line = LineString([(self.start_node.X, self.start_node.Y), (self.end_node.X, self.end_node.Y)])
        point = Point(x, y)
        distance = point.distance(line)
        return distance

    def get_traffic_count_on_road(self, road_network):

        # ToDo: Done: pre mark the lane bounding boxes/polygons of a given road. (take care of turns at intersections)
        # ToDo: the car should take the output of traffic on a lane as input and filter the cars ahead of it and behind
        # Note: shapely object.contains:- strictly inside the object. Does not include the boundary points
        # Note: to include boundary points, use object.intersects instead

        polygon = self.bounding_polygon
        bounding_box = self.min_bounding_box
        candidate_traffic = list(road_network.__car_tree.intersection(bounding_box))
        count = 0
        for candidate in candidate_traffic:
            point = Point(road_network.__cars_table[candidate].X, road_network.__cars_table[candidate].Y)
            if polygon.contains(point):
                count += 1

        return count

    def get_visible_cars_helper(self, car=None, polygon=None):

        visible_cars = []
        # candidate_traffic = self.total_traffic['cars']
        for candidate in self.total_traffic['cars']:
            if not candidate == car:
                point = Point(candidate.x, candidate.y)
                if polygon.contains(point):
                    visible_cars.append(candidate)

        return visible_cars

    def update_traffic(self, road_network):

        # returns a dict with key as lane_no and value as a list of cars in the lane
        # ToDo: working. Add more attributes to road.traffic

        lane_traffic = dict.fromkeys(range(0, self.no_of_lanes), [])
        lane_velocities = dict.fromkeys(range(0, self.no_of_lanes), [])
        lane_traffic_count = dict.fromkeys(range(0, self.no_of_lanes), 0)

        road_traffic = []
        road_velocities = []
        road_traffic_count = 0

        bounding_box = self.min_bounding_box
        candidate_traffic = list(road_network._car_tree.intersection(bounding_box))

        # ToDo: add a method to maintain a trace of the traffic. Estimate the traffic congestion from the trace
        # ToDo: Add a kalman filter to estimate the average velocity of the traffic on the road from the data
        # Note: We have the actual velocities of the vehicles, but we intend to not use the data.

        for candidate in candidate_traffic:
            point = Point(road_network._cars_table[candidate].x, road_network._cars_table[candidate].y)

            for lane in self.lanes:
                if lane.lane_polygon.contains(point):
                    lane_traffic_count[lane.lane_no] += 1
                    road_traffic_count += 1

                    lane_traffic[lane.lane_no].append(road_network._cars_table[candidate])
                    road_traffic.append(road_network._cars_table[candidate])

                    lane_velocities[lane.lane_no].append(road_network._cars_table[candidate].speed)
                    road_velocities.append(road_network._cars_table[candidate].speed)

        # self.traffic_count = road_traffic_count
        self.lane_wise_traffic['count'] = lane_traffic_count
        self.lane_wise_traffic['cars'] = lane_traffic
        self.lane_wise_traffic['average_velocity'] = sum(lane_velocities)/len(lane_velocities) \
            if len(lane_velocities) > 0 else 0
        self.lane_wise_traffic['velocities'] = lane_velocities

        self.total_traffic['count'] = road_traffic_count
        self.total_traffic['average_velocity'] = sum(road_velocities)/len(road_velocities) \
            if len(road_velocities) > 0 else 0
        self.total_traffic['velocities'] = road_velocities
        self.total_traffic['cars'] = road_traffic

        if self.total_traffic['count'] > 0:
            self.traffic_trace.append(self.total_traffic)


class TrafficGraph:
    def __init__(self):
        self.__graph = nx.DiGraph()
        self.__intersections_table = dict()
        self.__roads_table = dict()
        self.__sources = []
        self.__sinks = []
        self.__intersections_count = 0
        self.__nodes_count = 0
        self.__roads_count = 0

    def create_sample_network(self):
        self.__graph, self.__intersections_table, self.__roads_table = create_sample_network()
        self.__intersections_count = len(self.__intersections_table.keys())
        self.__roads_count = len(self.__roads_table.keys())
        for _, i in self.__intersections_table.items():
            self.__nodes_count += i.get_no_nodes()

    def get_intersections_table(self):
        return self.__intersections_table

    def get_all_nodes(self, objects=True):
        if objects:
            return self.__graph.nodes(data='object')
        else:
            return self.__graph.nodes()

    def get_shortest_path(self, source, destination):

        return nx.shortest_path(self.__graph, source, destination)

    def get_edges(self):

        return self.__graph.edges()

    def get_edge_attributes(self):

        return self.__graph.edges[edge]

    def get_road(self):

        return self.__graph.edges[edge]['object']

    def get_roads(self):

        return self.__graph.edges(data=True)

    def get_outgoing_edges(self):

        return self.__graph.out_edges(node)

    def get_incoming_edges(self):

        return self.__graph.in_edges(node)

    def get_adj_node(self):

        return self.__graph.adj[node]

    def get_node_attribute(self, node):

        return self.__graph[node]['object']


