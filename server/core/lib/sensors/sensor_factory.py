from .sensor import GPS, LaneCenterSensor, PositionBasedVelocityEstimator, BasicVisual


class SensorFactory:
    @staticmethod
    def build_sensor(type):
        if type == 'gps':
            return GPS()
        elif type == 'lane_center':
            return LaneCenterSensor()
        elif type == 'speed':
            return PositionBasedVelocityEstimator()
        elif type == 'basic_visual':
            return BasicVisual()

