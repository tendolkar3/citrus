from core.lib.maps.map import Intersection, Road, RoadString, Node, SubNode
import math
from shapely.geometry import Point, LineString, Polygon
from shapely.affinity import rotate
import networkx as nx


def convert_to_traffic_graph(G):

    intersections_table = dict()
    roads_table = dict()
    tg = nx.DiGraph()

    for n_start, value in G.succ.items():
        osmid = G._node[n_start]['osmid']
        x = G._node[n_start]['x']
        y = G._node[n_start]['y']
        if not osmid in intersections_table.keys():
            intersections_table[osmid] = ns = Intersection(point_x=x, point_y=y, osmid=osmid)
        else:
            ns = intersections_table[osmid]

        for n_end, road in value.items():
            osmid = G._node[n_end]['osmid']
            x = G._node[n_end]['x']
            y = G._node[n_end]['y']

            if not osmid in intersections_table.keys():
                intersections_table[osmid] = ne = Intersection(point_x=x, point_y=y, osmid=osmid)
            else:
                ne = intersections_table[osmid]
            R = road[0]
            name = R['name'] if 'name' in R.keys() else None
            if 'geometry' in R.keys():
                road_string = RoadString(R['geometry'])
            else:
                ls = LineString([(G._node[n_start]['x'],G._node[n_start]['y']),
                                 (G._node[n_end]['x'],G._node[n_end]['y'])])
                road_string = RoadString(ls)
            our_road, start_node, end_node = create_road(
                start_intersection=ns, end_intersection=ne, name=name, osmid=R['osmid'], road_string=road_string)

            tg.add_node(start_node.get_id(), object=start_node)
            tg.add_node(end_node.get_id(), object=end_node)
            tg.add_edges_from([(start_node.get_id(), end_node.get_id(), {'object': our_road,
                                                                         'distance': our_road.get_road_length(),
                                                                         'traffic': None})])
    # fixme: add turn roads connecting the roads across intersections


def create_road(start_intersection, end_intersection, name, osmid, road_string):
    """

    :param start_intersection:
    :param end_intersection:
    :param name:
    :param osmid:
    :param road_string:
    :return:
    """

    x1, y1 = start_intersection.get_x(), start_intersection.get_y()
    x2, y2 = end_intersection.get_x(), end_intersection.get_y()

    [p1, p2] = get_node_coords(x1, y1, x2, y2, no_of_lanes=3, lane_width=4)
    # todo: add utils for no of lanes and width

    # lane 0: (sn1, sn3)
    # lane 1: (sn2, sn4)
    id = str(start_intersection.get_osmid()) + "_" + str(osmid) + "_" + str(end_intersection.get_osmid())
    sn1 = Node(point_x=p1[0], point_y=p1[1], parent=start_intersection, _id=id)
    id = str(end_intersection.get_osmid()) + "_" + str(osmid) + "_" + str(start_intersection.get_osmid())
    sn2 = Node(point_x=p2[0], point_y=p2[1], parent=end_intersection, _id=id)

    road = Road(start_node=sn1, end_node=sn2, road_string=road_string, name=name, osmid=osmid)

    return road, sn1, sn2


def get_node_coords(x1, y1, x2, y2, no_of_lanes=3, lane_width=4):
    # os = (x1, y1)
    # oe = (x2, y2)
    road_width = no_of_lanes* lane_width
    distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    slope = math.atan2(y2 - y1, x2 - x1)
    fs = (0, 0)
    fe = (distance, 0)

    if abs(slope) <= math.pi / 2:
        # fs1 = Point(10, 0)
        fs2 = Point(10, -road_width/2)

        # fe1 = Point(distance - 10, 0)
        fe2 = Point(distance - 10, -road_width/2)

    else:
        # fs1 = Point(-10, 0)
        fs2 = Point(-10, -road_width/2)

        # fe1 = Point(-distance + 10, 0)
        fe2 = Point(-distance + 10, -road_width/2)

    os2 = rotate(fs2, angle=slope, origin=fs, use_radians=True)
    oe2 = rotate(fe2, angle=slope, origin=fs, use_radians=True)

    return [(x1 + os2.coords[0][0], y1 + os2.coords[0][1]), (x1 + oe2.coords[0][0], y1 + oe2.coords[0][1])]

