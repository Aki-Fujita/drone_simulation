from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt

"""
t = 0からスタートすることに注意！
"""
def conduct_fuel_optimization(**kwargs):
    x0 = kwargs.get("x0", 0)
    v0 = kwargs.get("v0", 0)
    xe = kwargs.get("xe", 0)
    te = kwargs.get("te", 0)
    a_max = kwargs.get("a_max", 0)
    a_min = kwargs.get("a_min", 0) # 許容減速度、正の数が来ることに注意！
    
    """
    燃料消費量の目的関数
    引数のaは加速度の配列
    """ 
    def fuel_consumption(a, dt):
        """
        計算用のパラメータを定義
        """
        k0 = 1  # 燃料消費関数の定数
        k1 = 0.1  # 燃料消費関数の速度に依存する定数
        k2 = 0.1  # 燃料消費関数の速度に依存する定数
        k3 = -1 # 最後の速度をなるべく大きくするために入れている
        # 時間ステップの大きさ
        v = np.zeros(N+1)
        x = np.zeros(N+1)
        v[0] = v0
        x[0] = x0
        
        fuel_cost = 0
        for i in range(1, N+1):
            v[i] = v[i-1] + a[i-1] * dt
            x[i] = x[i-1] + v[i-1] * dt + 0.5 * a[i-1] * dt**2
            fuel_cruise = k0 + k1 * v[i-1]**2
            fuel_acc = abs(k2 * a[i-1] * v[i-1])
            fuel_cost += (fuel_acc + fuel_cruise) * dt
        # 終了時の位置が目標値に一致するように大きなペナルティを追加
        penalty = 1e4 * (x[-1] - xe)**2
        v_e_bonus = k3 * v[-1]
        return fuel_cost + penalty + v_e_bonus


    for N in range(5, 40):
        a_initial = np.zeros(N)
        bounds = [(a_min, a_max) for _ in range(N)]
        dt = te / N
        result = minimize(fuel_consumption, a_initial, dt, method='SLSQP', bounds=bounds)
        print("=====")
        print(f"x_e = {xe},", calc_distance_from_a(result.x, x0, v0, dt, N))
        print(f"N = {N},", calc_distance_from_a(result.x, x0, v0, dt, N))
        print("=====")
        if abs(calc_distance_from_a(result.x, x0,v0, dt, N) - xe) < 1e-3:
            break

    print(result)
    a_optimized = result.x
    dt = te/N
    return a_optimized, dt, N
    
def calc_distance_from_a(a_optimized, x0, v0, dt, N):
    print(a_optimized)
    v = np.zeros(N+1)
    x = np.zeros(N+1)
    v[0] = v0
    x[0] = x0

    for i in range(1, N+1):
        v[i] = v[i-1] + a_optimized[i-1] * dt
        x[i] = x[i-1] + v[i-1] * dt + 0.5 * a_optimized[i-1] * dt**2
    return x[N]

if __name__ == "__main__":
    xe = 200  # 終了時の位置
    x0 = 10
    v0 = 12  # 初期速度
    te = 14  # 終了時刻
    a_max=4
    a_min=-4

    # 最適化の実行
    # result = minimize(fuel_consumption, a_initial, method='SLSQP', bounds=bounds)

    a_optimized, dt, N = conduct_fuel_optimization(
        x0=x0,
        v0=v0,
        xe=xe,
        te=te,
        a_max=a_max,
        a_min = a_min
    )

    v = np.zeros(N+1)
    x = np.zeros(N+1)
    v[0] = v0
    x[0] = x0

    for i in range(1, N+1):
        v[i] = v[i-1] + a_optimized[i-1] * dt
        x[i] = x[i-1] + v[i-1] * dt + 0.5 * a_optimized[i-1] * dt**2
    t = np.linspace(0, te, N+1)
    
    plt.figure(figsize=(10, 4))
    plt.plot(t, v, label='Velocity')
    plt.title('Velocity vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    plt.legend()
    plt.savefig("v-t.png")

    plt.figure(figsize=(10, 4))
    plt.plot(t[:-1], a_optimized, label='Acceleration')
    plt.title('Acc vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acc')
    plt.grid(True)
    plt.legend()
    plt.savefig("a-t.png")

    # 位置-時間グラフの描画
    plt.figure(figsize=(10, 4))
    plt.plot(t, x, label='Position', color='orange')
    plt.title('Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.grid(True)
    plt.legend()
    plt.savefig("x-t.png")


    plt.show()


