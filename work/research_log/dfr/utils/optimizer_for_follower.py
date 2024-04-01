from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt


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

def get_acc_for_time(data, t):
    """
    指定された時刻 t に対して、t 以下で最大の t_start を探し、その時の acc を返す。
    :param data: 辞書のリスト。各辞書は acc, t_start, v_0 のキーを持つ。
    :param t: 指定された時刻。
    :return: 条件を満たす acc の値。該当するものがなければ None を返す。
    """
    # t_start が t 以下の要素のみをフィルタリングし、t_start で降順にソート
    valid_items = sorted([item for item in data if item['t_start'] <= t], key=lambda x: x['t_start'], reverse=True)
    # 条件を満たす最初の要素の acc を返す
    return valid_items[0]['acc'] if valid_items else None 

"""
シミュレーションベースでやってぶつかったところから修正をかけていくようにする！
"""
def optimizer_for_follower(**kwargs):
    x0 = kwargs.get("x0", 0)
    v0 = kwargs.get("v0", 0)
    xe = kwargs.get("xe", 0)
    target_time = kwargs.get("target_time", 0)
    a_max = kwargs.get("a_max", 0)
    a_min = kwargs.get("a_min", 0) # 許容減速度、正の数が来ることに注意！
    leader = kwargs.get("leader", 0) # 先行車のオブジェクト
    current_time = kwargs.get("current_time", [])
    leader_acc_itinerary = leader.acc_itinerary
    leader_positions = [leader.xcor]
    leader_speeds = [leader.v_x] # ここは計算しないといけない！！
    print(leader.v_x, leader.xcor)

    # シミュレーションのパラメータ
    total_time = target_time - current_time
    time_step = 0.5
    steps = int(total_time / time_step)
    time_array = np.arange(time_step, total_time + time_step, time_step)
    print(target_time, current_time, leader_acc_itinerary)
    # 続いてleader_potisionsとleader_acc_itineraryを元にleaderの位置と速度を計算する
    for time in time_array:
        leader_acc = get_acc_for_time(leader_acc_itinerary, current_time + time)
        leader_speed = leader_speeds[-1]
        leader_speeds.append(leader_speed + leader_acc * time_step)
        leader_positions.append(leader_positions[-1] + leader_speed * time_step + 0.5 * leader_acc * time_step**2)
        # この時点でのleaderの情報を表示（その時間で動画を止めた時の位置などが出力される）
        print(current_time +time, leader_acc, leader_speeds[-1], leader_positions[-1])

    print("Leader Positions = \n",leader_positions)
    print("Leader Speeds = \n",leader_speed)

    ttc = 2

    # シミュレーションのパラメータ
    steps = 50
    time_step = total_time / steps
    time_array = np.arange(0, total_time, time_step)

    # 先頭車のパラメータ (一定速度)
    print("acc")
    print(leader.acc_itinerary)
    # 後続車のパラメータ
    follower_speed = 0  # 初期速度 (m/s)
    follower_position = [0]  # 初期位置 (位置のリスト)
    follower_acceleration = 0  # 初期加速度

    for i in range(1, len(time_array)):
        # 先頭車と後続車の間の距離
        distance = leader_position[i] - follower_position[-1]
        
        # 安全距離 (先頭車の速度に2秒間の時間を掛けた値)
        safe_distance = leader_speed * 2

        if distance > safe_distance + 5:  # 安全距離よりも十分遠い場合、加速
            follower_acceleration = 2  # 加速度 m/s^2
        elif distance > safe_distance:  # 安全距離よりもやや遠い場合、等速運転
            follower_acceleration = 0  # 加速度なし
        else:
            # 安全距離以下の場合、前のステップよりも加速度を小さくして減速
            follower_acceleration = max(-3, follower_acceleration - 1)  # 減速度の増加 (より強い減速)

        # 速度と位置の更新
        follower_speed += follower_acceleration * time_step
        follower_speed = max(follower_speed, 0)  # 速度が負にならないようにする
        new_position = follower_position[-1] + follower_speed * time_step
        follower_position.append(new_position)


    
if __name__ == "__main__":
    xe = 610  # 終了時の位置
    x0 = 0
    v0 = 20  # 初期速度
    te = 52  # 終了時刻
    a_max=3
    a_min=-3

    # 最適化の実行
    # result = minimize(fuel_consumption, a_initial, method='SLSQP', bounds=bounds)

    a_optimized, dt, N = optimizer_for_follower(
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


