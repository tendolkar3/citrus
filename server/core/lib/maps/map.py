from rtree import index
import networkx as nx
import itertools
import math
import numpy as np
# from core.lib.maps.utils import constants as c
from shapely.geometry import Polygon, LineString, Point
from core.lib.infrastructure.traffic import TrafficController
from core.helpers import norm, perpendicular_distance_of_point_from_line


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

    def get_traffic_controller(self):
        return self.__traffic_controller


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


class RoadNetwork:
    def __init__(self, graph=None):
        self.__road_tree = index.Index()
        self.__road_tree.interleaved = True
        self._cars_table = {}
        # ToDo: check python collections
        # ToDo: check memcache to store the data structures (trees and hash maps)
        self._car_tree = index.Index()
        self._car_tree.interleaved = True
        self.__graph = graph
        self.__node_count = 0
        self.__car_count = 0
        self.renderer = RoadNetworkRenderer()
        self.intersections_table = {}
        self.sources = []
        self.sinks = []

    def create_sample_road_network(self):

        # self.__graph = build_sample()
        self.__graph, self.intersections_table = build_sample()
        self.__build_rtree()
        for source in [9, 25, 27, 43, 45, 61, 77, 79, 95, 89]:
            self.sources.append(self.get_node_attr(source))
        for sink in [10, 26, 28, 44, 46, 62, 78, 80, 96, 90]:
            self.sinks.append(self.get_node_attr(sink))

    def __build_rtree(self):

        for node in self.__graph.nodes(data='object'):
            temp_id = node[1].id
            temp_x = node[1].X
            temp_y = node[1].Y
            self.__road_tree.insert(temp_id, (temp_x, temp_y, temp_x, temp_y))
            self.__node_count += 1

            # have to decide if obj=node[1] should be included or not

    def get_nearest_nodes(self, bounding_box, number, generator=True, objects=False):

        if generator:
            return self.__road_tree.nearest(bounding_box, number, objects)
        else:
            return list(self.__road_tree.nearest(bounding_box, number, objects))

    def get_nodes_by_bounding_box(self, bounding_box, generator=True, objects=False):

        if generator:
            return self.__road_tree.nearest(bounding_box, objects)
        else:
            return list(self.__road_tree.nearest(bounding_box, objects))

    def get_nearest_cars(self, bounding_box, number, generator=True, objects=False):

        if generator:
            return self._car_tree.nearest(bounding_box, number, objects)
        else:
            return list(self._car_tree.nearest(bounding_box, number, objects))

    def get_cars_by_bounding_box(self, bounding_box, generator=True, objects=False):

        if generator:
            return self._car_tree.intersection(bounding_box, objects)
        else:
            return list(self._car_tree.intersection(bounding_box, objects))

    def get_all_nodes(self, objects=True):

        if objects:
            return self.__graph.nodes(data='object')
        else:
            return self.__graph.nodes()

    def get_shortest_path(self, source, destination):

        return nx.shortest_path(self.__graph, source, destination)

    def get_edges(self):

        # returns a EdgeView containing edge. For attributes call getEdgeAttr(edge)
        return self.__graph.edges()

    def get_edge_attr(self, edge):

        # returns a dict of the attributes
        return self.__graph.edges[edge]

    def get_road(self, edge):

        return self.__graph.edges[edge]['object']

    def get_roads(self):

        return self.__graph.edges(data=True)

    def get_outgoing_edges(self, node):

        return self.__graph.out_edges(node)

    def get_incoming_edges(self, node):

        return self.__graph.in_edges(node)

    # def get_adj_edge_of_node(self, node):

        # returns a dict (AtlasView).The key is the node connecting the given node
        # return self.__graph[node]

    def get_adj_nodes(self, node):

        return self.__graph.adj[node]

    def get_node_attr(self, node):

        return self.__graph.node[node]['object']

    def get_node_count(self):

        return self.__node_count

    def get_car_count(self):

        return len(self._cars_table.keys())
        # return self.__car_count

    def get_road_network_bbox(self):

        return self.__road_tree.bounds

    def get_car_tree_bbox(self):
        try:
            return self._car_tree.bounds
        except:
            return None

    def update_traffic(self, road):
        # Wrote the same method for a road. Duplicating it for a roadnetwork here, not using this anywhere as of now
        # resolve the name conflict later

        lane_traffic = dict.fromkeys(range(0, road.no_of_lanes), [])
        road_traffic_count = 0
        bounding_box = road.min_bounding_box
        # print(bounding_box)
        candidate_traffic = list(self._car_tree.intersection(bounding_box))

        for candidate in candidate_traffic:
            point = Point(self._cars_table[candidate].x, self._cars_table[candidate].y)

            for lane in road.lanes:
                if lane.lane_polygon.contains(point):
                    road_traffic_count += 1
                    lane_traffic[lane.lane_no].append(self._cars_table[candidate])
                    lane.traffic.append(self._cars_table[candidate])

        road.traffic_count = road_traffic_count
        road.traffic = lane_traffic

    def get_traffic_count_on_road(self, road):

        # ToDo: check for test cases get_traffic_on_road()
        polygon = road.get_bounding_polygon()
        bounding_box = polygon.bounds
        candidate_traffic = list(self._car_tree.intersection(bounding_box))
        count = 0
        for candidate in candidate_traffic:
            point = Point(self._cars_table[candidate].x, self._cars_table[candidate].y)
            if polygon.contains(point):
                count += 1

        return count

    def add_car(self, car):

        # if self._car_tree is None

        # if car.ID is not None:
        # car.ID = self.__car_count + 1
        self.__car_count += 1

        self._cars_table[car.ID] = car
        self._car_tree.insert(car.ID, (car.x, car.y, car.x, car.y))

    def build_car_tree(self, cars):

        count = self.__car_count
        counter = itertools.count(start=count + 1)
        # itertools counter to generate a unique id to every new car inserted
        for car in cars:
            if car.ID is None:
                car.ID = next(counter)
                self.__car_count += 1

            self.__car_count += 1

            self._cars_table[car.ID] = car

        self._car_tree = index.Index(self.car_generator())

    def get_cars(self):

        # returns a tuple containing the carid and the car object
        for _, value in self._cars_table.items():
            yield value

    def get_car(self, car_id):

        return self._cars_table[car_id]

    def remove_car(self, car):

        self.__car_count -= 1
        self._car_tree.delete(car.ID, (car.x, car.y, car.x, car.y))
        del self._cars_table[car.ID]

    def car_generator(self):
        for id_, car in self._cars_table.items():
            yield (id_, (car.x, car.y, car.x, car.y), None)

    def update_car_network(self):
        # Used bulk upload of data into rtree. Much more faster than individual upload

        # del self._car_tree
        # self._car_tree = index.Index()
        # self._car_tree.interleaved = True
        #
        # for id_, car in self._cars_table.items():
        #     self._car_tree.insert(id_, (car.x, car.y, car.x, car.y))

        del self._car_tree
        self._car_tree = index.Index(self.car_generator())

    def update_road_attributes(self, start_node_id, end_node_id, road):

        features = calculate_traffic_features(road)
        # Should return a dict, with the following keys
        features_list = ['congestion', 'estimated_fuel_usage', 'estimated_time', 'distance']
        # Decide on how to calculate these features.
        for feature in features_list:
            self.__graph[start_node_id][end_node_id][feature] = features[feature]


def build_sample_1():
    graph = nx.DiGraph(name='test_graph')
    x = itertools.count()
    intersection, x = create_plus_intersection(100, 100, x, scale=30)
    graph = add_intersection_to_graph(graph, [intersection])

    return graph


def build_sample():
    graph = nx.DiGraph(name='test_graph')
    x = itertools.count(1)

    i1, x = create_plus_intersection(100, 350, x, scale=45)
    i2, x = create_plus_intersection(100, 100, x, scale=45)
    i3, x = create_plus_intersection(350, 100, x, scale=45)
    i4, x = create_plus_intersection(350, 350, x, scale=45)
    i5, x = create_plus_intersection(350, 600, x, scale=45)
    i6, x = create_plus_intersection(100, 600, x, scale=45)

    intersections_table = dict()
    intersections_table[1] = i1
    intersections_table[2] = i2
    intersections_table[3] = i3
    intersections_table[4] = i4
    intersections_table[5] = i5
    intersections_table[6] = i6

    graph = add_intersection_to_graph(graph, [i1, i2, i3, i4, i5, i6])

    links = [[32, 11], [12, 31], [30, 41], [42, 29], [48, 59], [60, 47], [58, 13], [14, 57], [16, 91], [92, 15],
             [64, 75], [76, 63], [74, 93], [94, 73]]

    for link in links:
        temp_node1 = graph.node[link[0]]['object']
        temp_node2 = graph.node[link[1]]['object']
        temp_road = Road(start_node=temp_node1, end_node=temp_node2)
        graph.add_edges_from([(temp_node1.id, temp_node2.id, {'object': temp_road, 'weight': temp_road.length})])

    return graph, intersections_table


def build_random():
    graph = nx.DiGraph(name='random')

    return graph


def create_plus_intersection(point_x, point_y, counter, scale=30):

    traffic_controller = TrafficController()

    intersection = Intersection(point_x, point_y, int_type='plus', traffic_controller=traffic_controller)
    # print(intersection.traffic_controller.signals)

    intersection.nodes_append(Node(point_x - 1 * scale, point_y + 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 1 * scale, point_y - 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 0.5 * scale, point_y - 1 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 0.5 * scale, point_y - 1 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 1 * scale, point_y - 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 1 * scale, point_y + 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 0.5 * scale, point_y + 1 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 0.5 * scale, point_y + 1 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 2 * scale, point_y + 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 2 * scale, point_y - 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 0.5 * scale, point_y - 2 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 0.5 * scale, point_y - 2 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 2 * scale, point_y - 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 2 * scale, point_y + 0.5 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x + 0.5 * scale, point_y + 2 * scale, id_=next(counter),
                                   parent=intersection))
    intersection.nodes_append(Node(point_x - 0.5 * scale, point_y + 2 * scale, id_=next(counter),
                                   parent=intersection))

    return intersection, counter


def add_intersection_to_graph(graph, intersections):
    # ToDo: Done: check the lane width and no of lanes for roads representing turns at intersection
    # ToDo: perfect the addition of traffic signals. Now doing a handwavy implementation

    for I in intersections:
        for node in I.get_nodes():
            graph.add_node(node.id, object=node)
        turn_links = [[0, 3], [0, 5], [0, 7], [2, 5], [2, 7], [2, 1], [4, 7], [4, 1], [4, 3], [6, 1], [6, 3], [6, 5]]
        straight_links = [[8, 0], [10, 2], [12, 4], [14, 6], [1, 9], [3, 11], [5, 13], [7, 15]]

        for link in turn_links:
            temp_node1 = I.get_nodes(link[0])
            temp_node2 = I.get_nodes(link[1])
            temp_road = Road(start_node=temp_node1, end_node=temp_node2, _type='turn')
            graph.add_edges_from([(temp_node1.id, temp_node2.id, {'object': temp_road, 'weight': temp_road.length})])

        for link in straight_links:
            temp_node1 = I.get_nodes(link[0])
            temp_node2 = I.get_nodes(link[1])
            # This is a temporary code. We need to make it more general for any kind of intersection.
            if link[1] in [0, 2, 4, 6]:
                # print(type(I.traffic_controller.signals))
                temp_node2.traffic_signals[0] = I.traffic_controller.signals[int(3 * link[1]/2)]
                I.traffic_controller.signals[int(3 * link[1]/2)].set_node(temp_node2)
                temp_node2.traffic_signals[1] = I.traffic_controller.signals[int(3 * link[1]/2) + 1]
                I.traffic_controller.signals[int(3 * link[1]/2) + 1].set_node(temp_node2)
                temp_node2.traffic_signals[2] = I.traffic_controller.signals[int(3 * link[1]/2) + 2]
                I.traffic_controller.signals[int(3 * link[1]/2) + 2].set_node(temp_node2)

                # print(temp_node2.id, temp_node2.X, temp_node2.Y)

            temp_road = Road(start_node=temp_node1, end_node=temp_node2)
            graph.add_edges_from([(temp_node1.id, temp_node2.id, {'object': temp_road, 'weight': temp_road.length})])

    return graph


def polygon_from_linestring(linestring, width_from_center):

    pts = []
    for x, y in zip(linestring.coords.xy[0], linestring.coords.xy[1]):
        pts.append([x, y])

    theta = math.atan2((pts[1][1] - pts[0][1]), (pts[1][0] - pts[0][0]))
    w = width_from_center
    pt1 = (pts[0][0] - w * math.sin(theta), pts[0][1] + w * math.cos(theta))
    pt2 = (pts[0][0] + w * math.sin(theta), pts[0][1] - w * math.cos(theta))

    theta = math.atan2(-(pts[1][1] - pts[0][1]), -(pts[1][0] - pts[0][0]))
    pt3 = (pts[1][0] - w * math.sin(theta), pts[1][1] + w * math.cos(theta))
    pt4 = (pts[1][0] + w * math.sin(theta), pts[1][1] - w * math.cos(theta))

    polygon = Polygon([pt1, pt2, pt3, pt4])

    return polygon
