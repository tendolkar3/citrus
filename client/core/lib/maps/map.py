from rtree import index
import networkx as nx
import itertools
import math
import numpy as np
from core.lib.maps.utils import constants as c
from shapely.geometry import Polygon, LineString, Point
from core.lib.infrastructure.traffic import TrafficController
from core.helpers import norm, perpendicular_distance_of_point_from_line


class Node:
    def __init__(self, point_x, point_y, grade=0, id_=None, parent=None):
        self._X = point_x
        self._Y = point_y
        self._grade = grade
        self._parent = parent
        self._id = id_
        self._type = 'node'
        self._traffic_signals = dict().fromkeys(range(3))


class Lane:
    def __init__(self, polygon, line_string, lane_no, _type=None, road=None, speed_limit=c.SPEED_LIMIT):

        self.lane_polygon = polygon
        self.lane_center = line_string
        self.lane_no = lane_no
        self.lane_type = _type
        self.road = road
        self.speed_limit = speed_limit
        self.name = str(self.road.name)+"-lmsp-" + str(self.speed_limit)
        x, y = self.lane_center.coords.xy
        self.start_x = x[0]
        self.start_y = y[0]
        self.end_x = x[-1]
        self.end_y = y[-1]


class Road:
    def __init__(self, start_node=None, end_node=None, _type='straight', no_of_lanes=c.NO_OF_LANES,
                 lane_width=c.LANE_WIDTH, speed_limit=c.SPEED_LIMIT):
        self.start_node = start_node
        self.end_node = end_node
        self.type = _type
        self.speed_limit = speed_limit
        self.heading = 0
        self.length = 0
        self.traffic_count = 0
        self.traffic_trace = []
        self.lane_wise_traffic = {'count': 0, 'cars': [], 'average_velocity': 0, 'variance_velocity': 0, 'velocities': []}
        self.total_traffic = {'count': 0, 'cars': [], 'average_velocity': 0, 'variance_velocity': 0, 'velocities': []}
        # traffic_trace is a dict of the last 50 time interval's traffic data. At each time interval, it is a dict with
        # the above given keys, each of whose values return an empty list

        # ToDo: add more traffic attributes of the road like traffic traces, congestion parameters etc.
        self.no_of_lanes = no_of_lanes
        self.lanes = []
        self.lane_width = lane_width
        self.min_bounding_box = None
        self.bounding_polygon = None
        self.name = str(self.start_node.id) + "-" + str(self.end_node.id) + "-msp-" + str(self.speed_limit)
        self.__calculate__()

    def __calculate__(self):
        diff = np.array([self.start_node.X, self.start_node.Y]) - np.array([self.end_node.X, self.end_node.Y])
        self.length = norm(diff)
        self.heading = math.atan2((self.end_node.Y - self.start_node.Y), (self.end_node.X - self.start_node.X))
        self.min_bounding_box = self.get_min_bounding_box()
        self.bounding_polygon = self.get_bounding_polygon()
        self.__lane_polygons()

    def get_bounding_polygon(self):
        # The bounding polygon for turn roads are not consistent. The polygon partly overlaps with the adjacent straight
        # roads. For now this is fine, but this should be corrected later.

        # theta = math.atan2((self.end_node.Y-self.start_node.Y), (self.end_node.X-self.start_node.X))
        theta = self.heading
        w = (self.lane_width * self.no_of_lanes)/2
        pt1 = (self.start_node.X - w * math.sin(theta), self.start_node.Y + w * math.cos(theta))
        pt2 = (self.start_node.X + w * math.sin(theta), self.start_node.Y - w * math.cos(theta))

        theta = math.atan2(-(self.end_node.Y-self.start_node.Y), -(self.end_node.X-self.start_node.X))
        pt3 = (self.end_node.X - w * math.sin(theta), self.end_node.Y + w * math.cos(theta))
        pt4 = (self.end_node.X + w * math.sin(theta), self.end_node.Y - w * math.cos(theta))

        # print(self.start_node.id, self.end_node.id, pt1, pt2, pt3, pt4)

        polygon = Polygon([pt1, pt2, pt3, pt4])

        return polygon

    def get_min_bounding_box(self):

        polygon = self.get_bounding_polygon()
        # print(self.start_node.id, self.end_node.id, polygon.bounds)
        return polygon.bounds

    def __lane_polygons(self):
        # ToDo: Done: make test cases to check if the lane polygons are correct

        center_linestring = LineString([(self.start_node.X, self.start_node.Y), (self.end_node.X, self.end_node.Y)])

        if self.no_of_lanes % 2 == 0:
            for ind in range(1, 1 + self.no_of_lanes//2):
                distance = self.lane_width/2 + self.lane_width * (self.no_of_lanes/2 - ind)
                left_linestring = center_linestring.parallel_offset(distance, 'left')
                poly = polygon_from_linestring(left_linestring, self.lane_width/2)
                lane = Lane(polygon=poly, line_string=left_linestring, lane_no=ind - 1, road=self)
                self.lanes.append(lane)

            for ind in range(self.no_of_lanes//2):
                distance = self.lane_width/2 + self.lane_width * ind
                right_linestring = center_linestring.parallel_offset(distance, 'right')
                poly = polygon_from_linestring(right_linestring, self.lane_width/2)
                lane = Lane(polygon=poly, line_string=right_linestring, lane_no=self.no_of_lanes//2 + ind, road=self)
                self.lanes.append(lane)

        else:
            for ind in range(1, 1 + self.no_of_lanes // 2):
                distance = self.lane_width * (1 + self.no_of_lanes // 2 - ind)
                # left_linestring = center_linestring.parallel_offset(distance, 'left')
                right_linestring = center_linestring.parallel_offset(distance, 'right')
                x, y = right_linestring.coords.xy
                right_linestring = LineString([(x[-1], y[-1]), (x[0], y[0])])

                poly = polygon_from_linestring(right_linestring, self.lane_width / 2)
                lane = Lane(polygon=poly, line_string=right_linestring, lane_no=ind - 1, road=self)
                self.lanes.append(lane)

            poly = polygon_from_linestring(center_linestring, self.lane_width / 2)
            lane = Lane(polygon=poly, line_string=center_linestring, lane_no=self.no_of_lanes//2, road=self)
            self.lanes.append(lane)

            # ToDo: Shapely issue #284: parallel offset to the right returns a line string in the reverse order.
            # ToDo: Did not find a quick fix, so manually reversing it after the parallel offset

            for ind in range(self.no_of_lanes // 2):
                distance = self.lane_width + self.lane_width * ind
                left_linestring = center_linestring.parallel_offset(distance, 'left')
                # right_linestring = center_linestring.parallel_offset(distance, 'right')
                # x, y = right_linestring.coords.xy
                # right_linestring = LineString([(x[-1], y[-1]), (x[0], y[0])])
                poly = polygon_from_linestring(left_linestring, self.lane_width / 2)
                lane = Lane(polygon=poly, line_string=left_linestring, lane_no=self.no_of_lanes//2 + ind + 1,
                            road=self)
                self.lanes.append(lane)

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


class Intersection:
    def __init__(self, point_x, point_y, id_=None, int_type=None, traffic_controller=None):
        self.X = point_x
        self.Y = point_y
        self.__nodes = []
        self.type = int_type
        self.id = id_
        self.traffic_controller = traffic_controller

    def nodes_append(self, node):
        self.__nodes.append(node)

    def get_nodes(self, ind=None):
        if ind is not None:
            return self.__nodes[ind]
        else:
            return self.__nodes


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
