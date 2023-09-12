import math


def solve_quadratic(a, b, c):
    # 判別式を計算
    discriminant = b**2 - 4 * a * c

    # 判別式が正の場合
    if discriminant > 0:
        x1 = (-b + math.sqrt(discriminant)) / (2 * a)
        x2 = (-b - math.sqrt(discriminant)) / (2 * a)
        return x1, x2

    # 判別式が0の場合
    elif discriminant == 0:
        x1 = -b / (2 * a)
        return [x1, x1]

    # 判別式が負の場合 (実数解は存在しない)
    else:
        return [None, None]


def find_delta_x_list(CARS):
    xcor_list = [car.xcor for car in CARS]
    delta_x_list = [xcor_list[int(i - 1)] - xcor_list[int(i)] if i > 0 else 1e4 for i in range(len(xcor_list))]
    return delta_x_list


def find_delta_v_list(CARS):
    v_list = [car.v_x for car in CARS]
    delta_v_list = [v_list[int(i - 1)] - v_list[int(i)] if i > 0 else 1e4 for i in range(len(v_list))]
    return delta_v_list
