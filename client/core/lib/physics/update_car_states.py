import math
from core.lib.maps import constants as road_c
from core.config import sim_c


def two_wheel_drive(x, y, heading, speed, length, steering_angle, gas, brake, gas_to_acc=1, brake_to_acc=1):
    """
    This method follows a front two wheel drive model
    :param x: car's x coordinate
    :param y: car's y coordinate
    :param heading: car's heading
    :param speed: car's speed
    :param length: car's length
    :param steering_angle: car's steering angle
    :param gas: car's gas
    :param brake: car's brake
    :param gas_to_acc: car's gas to acceleration constant
    :param brake_to_acc: car's brake to acceleration constant
    :return: car's updated x, y, heading, speed
    """

    front_wheel_x = x + length / 2 * math.cos(heading)
    front_wheel_y = y + length / 2 * math.sin(heading)
    back_wheel_x = x - length / 2 * math.cos(heading)
    back_wheel_y = y - length / 2 * math.sin(heading)

    speed += (
        gas * gas_to_acc * sim_c.DT - (
            brake * brake_to_acc * sim_c.DT) - road_c.DRAG_COEF * speed * sim_c.DT)
    speed = speed if speed > 0 else 0

    # update wheel positions
    front_wheel_x += speed * c.DT * math.cos(heading + steering_angle)
    front_wheel_y += speed * c.DT * math.sin(heading + steering_angle)
    back_wheel_x += speed * c.DT * math.cos(heading)
    back_wheel_y += speed * c.DT * math.sin(heading)

    # update car position and heading
    x = (front_wheel_x + back_wheel_x) / 2
    y = (front_wheel_y + back_wheel_y) / 2
    heading = math.atan2((front_wheel_y - back_wheel_y), (front_wheel_x - back_wheel_x))

    return x, y, heading, speed


def four_wheel_drive(x, y, heading, speed, length, steering_angle, gas, brake, gas_to_acc=1, brake_to_acc=1):
    """
    This method follows a four wheel drive model
    :param x: car's x coordinate
    :param y: car's y coordinate
    :param heading: car's heading
    :param speed: car's speed
    :param length: car's length
    :param steering_angle: car's steering angle
    :param gas: car's gas
    :param brake: car's brake
    :param gas_to_acc: car's gas to acceleration constant
    :param brake_to_acc: car's brake to acceleration constant
    :return: car's updated x, y, heading, speed
    """

    return x, y, heading, speed
