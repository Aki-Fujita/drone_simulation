# 参考 https://qiita.com/jabberwocky0139/items/e2526fc5ee3b0dbf144b
# import numpy as np

"""
a: 感応度
c: 適当な定数（加速しない車間距離がこれで決まる）
delta_x: 前の車との車間距離
delta_t:時間幅,
v_n: 現在の速度
"""


def helly(delta_x, delta_v, delta_t, v_n, helly_params, car_idx):
    max_accel = helly_params.get("max_accel")
    min_accel = helly_params.get("min_accel")
    lambda_1 = helly_params.get("lambda_1")
    lambda_2 = helly_params.get("lambda_2")
    d = helly_params.get("d")
    T_des = helly_params.get("T_des")
    isRss = helly_params.get("isRss", False)

    # 安全距離の式
    def D(v, isRSS=False, should_print=False):
        if isRSS:
            rho_delay = helly_params.get("response_time", 0.5)
            min_comfortable_accel = helly_params.get("rear_brake_acc", min_accel*0.5)
            front_car_brake = helly_params.get("front_car_brake", min_accel)
            proceeding_speed = v + delta_v
            front_car_brake_distance = proceeding_speed**2 / front_car_brake / 2
            brake_distance = (v + max_accel*rho_delay)**2/(min_comfortable_accel)/2
            idle_distance = v * rho_delay + max_accel * rho_delay**2 / 2

            # if (car_idx == 1 and should_print and v > 1):
            #     print("my speed:", v)
            #     print("proceeding_speed:", proceeding_speed)
            #     print("front_car_brake_distance:", front_car_brake_distance)
            #     print("brake_distance:", brake_distance)
            #     print("idle_distance:", idle_distance)
            return d + brake_distance + idle_distance - front_car_brake_distance
        return d + T_des * v

    # 更新式
    def f(v):
        return lambda_1 * (delta_x - D(v, isRss)) + lambda_2*(delta_v)

    # ここからルンゲクッタ
    k1 = f(v_n)
    k2 = f(v_n + k1 * delta_t / 2)
    k3 = f(v_n + k2 * delta_t / 2)
    k4 = f(v_n + k3 * delta_t)
    # if (car_idx == 1):
    #     print("RSS距離=", D(v_n, isRss, True))
    #     print("my speed:", v_n)

    desired_acceleration = (k1 + 2*k2 + 2*k3 + k4) / 6
    # print("delta_x=", delta_x, ", D=", D(v_n, isRss))

    # 加速の場合
    if desired_acceleration >= 0:
        # print("加速", max_accel, desired_acceleration, min(desired_acceleration, max_accel) * delta_t + v_n)
        return min(desired_acceleration, max_accel) * delta_t + v_n
    # 減速の場合
    # print("減速", -1*min_accel, desired_acceleration)

    deceleration = max(desired_acceleration, -1*min_accel)
    return max(deceleration * delta_t + v_n, 0)
