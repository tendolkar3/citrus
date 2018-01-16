import sys
import time
from core.lib.maps.map import Net
from core.lib.vehicles.vehicle_factory import VehicleFactory
import os
from core.lib.physics.utils import constants as phy_c
from random import choice
import pickle
# ToDo: use @jitclass(spec) for all the classes used. Make a list of the "specs" for all the classes"

dir_path = os.path.dirname(os.path.realpath(__file__))


# @jitclass(spec)
class Simulation(object):
    def __init__(self):

        self.clock = pg.time.Clock()
        self.paused = False

        # Create a pre-defined road network
        self.road_net = RoadNetwork()
        self.road_net.create_sample_road_network()

        NUM_CARS = 50
        # NUM_CARS = 10
        init_speeds = list(range(5,15))

        self.cars = []

        for i in range(1, NUM_CARS):
            first = choice(self.road_net.sources)
            # print(first.X, first.Y, first.id)
            last = choice(self.road_net.sinks)
            route = self.road_net.get_shortest_path(first.id, last.id)
            route = route[::-1]
            c = DisplayCar(screen=self.screen, image=car_image, idx=i, name="Tesla",
                           x=first.X, y=first.Y, start_time=i*1,
                           lane_center_dy=0, lane_center_dtheta=0,
                           speed=choice(init_speeds), acceleration=0, heading=0,
                           length=phy_c.CAR_LENGTH, width=phy_c.CAR_WIDTH,
                           route=route, road_net=self.road_net)
            self.cars.append(c)
        # self.road_net.build_car_tree(cars)

        first = self.road_net.sources[0]
        last = self.road_net.sinks[0]
        route = self.road_net.get_shortest_path(first.id, last.id)
        route = route[::-1]
        test_car = DisplayCar(screen=self.screen, image=car_image, idx=0, name='Testla', x=first.X, y=first.Y,
                              start_time=0, lane_center_dy=0, lane_center_dtheta=0, speed=choice(init_speeds),
                              acceleration=0, heading=0, length=phy_c.CAR_LENGTH + 5, width=phy_c.CAR_WIDTH,
                              route=route,
                              road_net=self.road_net)
        self.cars.append(test_car)

    # @jit(nopython=True)
    def run(self):
        # Main Simulation Loop
        simulation_start_time = time.time()
        counter = 0

        while True:
            if counter % 30 == 0:
                if len(self.cars) > 0:
                    # print("add")
                    car = self.cars.pop()
                    self.road_net.add_car(car)
            counter += 1

            # Limit frame speed to 30 FPS
            time_passed = self.clock.tick(30)
            # ~ time_passed = self.clock.tick()
            # ~ print time_passed
            # If too long has passed between two frames, don't
            # update (the simulation must have been suspended for some
            # reason, and we don't want it to "jump forward"
            # suddenly)
            if time_passed > 100:
                continue

            for intersection in self.road_net.intersections_table.values():
                tc = intersection.traffic_controller
                tc.update_signals()
                # for signal in tc.signals.values():
                #     print(signal.tcid, signal.get_state())
                # updates the traffic signal state at each junction/intersection

            for road in self.road_net.get_roads():
                road[2]['object'].update_traffic(self.road_net)
                # features = calculate_traffic_features(road[2]['object'])
                self.road_net.update_road_attributes(road[0], road[1], road[2])
                # if road[2]['object']
                # road.traffic is a dict with lane number as keys
                # The above method get_traffic_by_lane() also updates the traffic of each lane in lane.traffic

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.quit()
                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        self.paused = not self.paused

            if not self.paused:
                # Update and all cars
                ids_to_delete = []
                for car in self.road_net.get_cars():
                    if len(car.route) == 1:
                        ids_to_delete.append(car.ID)
                        pickle.dump(car.speed_trace, open("data/car_"+str(car.ID)+".p","wb"))
                        pickle.dump(car.steering_trace, open("data/steering_"+str(car.ID)+".p","wb"))
                for id_ in ids_to_delete:
                    self.road_net.remove_car(self.road_net.get_car(id_))

                for car in self.road_net.get_cars():
                    # if car.start_time < (time.time()-simulation_start_time):
                    car.update(self.road_net)

                if self.road_net.get_car_count() > 0:
                    self.road_net.update_car_network()
                    # print(self.road_net.get_car_count())
                else:
                    self.paused = True

                    print("DONE")
                    sys.exit()

            else:
                for road in self.road_net.get_roads():
                    # print(road[0],road[1])
                    if len(road[2]['object'].traffic_trace) > 0:
                        pickle.dump(road[2]['object'].traffic_trace,
                                    open("data/road_" + str(road[0]) + str(road[1]) + ".p", "wb"))
                sys.exit()

            pg.display.flip()
            # print("NEW")

    def quit(self):
        sys.exit()


if __name__ == "__main__":
    sim = Simulation()
    # cProfile.run('sim.run()')
    sim.run()