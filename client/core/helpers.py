import math


def norm(point):

    x, y = point
    return math.sqrt(x**2 + y**2)


def perpendicular_distance_of_point_from_line(point, line_parametric=None, line_points=None):

    if line_parametric is None:
        if line_points is not None:
            # calculates the parametric line
            [(x1, y1), (x2, y2)] = line_points
            if x1 == x2:
                a, b, c = 1, 0, -x1
            else:
                m = (y2-y1)/(x2-x1)
                a = m
                b = -1
                c = y1 - m*x1

        else:
            return None
    else:
        [a, b, c] = line_parametric

    [x, y] = point
    distance = abs((a*x + b*y + c)/math.sqrt(a**2+b**2))

    # print(point, line_points, distance, a, b, c)

    return distance
