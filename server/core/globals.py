from core.lib.maps.map import RoadNetwork as Net


def init():

    global map_2d
    map_2d = Net()
    # ToDo: Take parameters from a config file to load an existing map
