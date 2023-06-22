# 参考 https://qiita.com/jabberwocky0139/items/e2526fc5ee3b0dbf144b
# import numpy as np

"""
a: 感応度
c: 適当な定数（加速しない車間距離がこれで決まる）
delta_x: 前の車との車間距離
delta_t:時間幅,
v_n: 現在の速度
"""


def helly(delta_x, delta_v, delta_t, v_n, helly_params):
    max_accel, min_accel, lambda_1, lambda_2, d, T_des, isRss = helly_params.values()

    # 安全距離の式
    def D(v, isRSS=False):
        if isRSS:
            rho_delay = 1
            front_car_max_brake = min_accel
            proceeding_speed = v - delta_v
            front_car_brake_distance = proceeding_speed**2 / front_car_max_brake / 2
            brake_distance = (v+max_accel*rho_delay)**2/(max_accel)/2
            idle_distance = v * rho_delay + max_accel * rho_delay**2 / 2
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

    v_diff = delta_t * (k1 + 2*k2 + 2*k3 + k4) / 6

    # 加速の場合
    if v_diff >= 0:
        # print(v_diff, max_accel)
        return min(v_diff, max_accel * delta_t) + v_n
    # 減速の場合
    return max(max(v_diff, -1*min_accel * delta_t) + v_n, 0)   
