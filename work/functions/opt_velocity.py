# 参考 https://qiita.com/jabberwocky0139/items/e2526fc5ee3b0dbf144b
import numpy as np

"""
a: 感応度
c: 適当な定数（加速しない車間距離がこれで決まる）
delta_x: 前の車との車間距離
"""


def optimal_velocity(c, a, delta_x):

    # 最適速度関数
    def V(h):
        return np.tanh(h - c) + np.tanh(c)

    # 更新式
    def f(v_n):
        return a * (V(delta_x) - v_n)

    # ここからルンゲクッタ
    k1 = f()
