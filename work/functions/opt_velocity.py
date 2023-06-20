# 参考 https://qiita.com/jabberwocky0139/items/e2526fc5ee3b0dbf144b
import numpy as np

"""
a: 感応度
c: 適当な定数（加速しない車間距離がこれで決まる）
delta_x: 前の車との車間距離
delta_t:時間幅,
v_n: 現在の速度
"""


def optimal_velocity(c, a, delta_x, delta_t, v_n):
    # 最適速度関数
    def V(h):
        return np.tanh(h - c) + np.tanh(c)

    # 更新式
    def f(v):
        return a * (V(delta_x) - v)

    # ここからルンゲクッタ
    k1 = f(v_n)
    k2 = f(v_n + k1 * delta_t / 2)
    k3 = f(v_n + k2 * delta_t / 2)
    k4 = f(v_n + k3 * delta_t)

    return v_n + delta_t * (k1 + 2 * k2 + 2 * k3 + k4) / 6
