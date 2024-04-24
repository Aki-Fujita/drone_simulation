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
    valid_items = sorted([item for item in data if item['t_start'] < t], key=lambda x: x['t_start'], reverse=True)
    # 条件を満たす最初の要素の acc を返す
    return valid_items[0]['acc'] if valid_items else None 

"""
シミュレーションベースでやってぶつかったところから修正をかけていくようにする！
"""
def optimizer_for_follower(**kwargs):
    follower=kwargs.get("follower")
    x0 = follower.xcor
    xe = kwargs.get("xe", 0)
    target_time = kwargs.get("target_time", 0) 
    a_max = kwargs.get("a_max", 0)
    a_min = kwargs.get("a_min", 0) # 許容減速度、正の数が来ることに注意！
    ttc = kwargs.get("ttc", 0)
    ttc += 0.5 # 一旦これで応急処置
    leader = kwargs.get("leader", 0) # 先行車のオブジェクト
    current_time = kwargs.get("current_time", [])
    leader_acc_itinerary = leader.acc_itinerary
    leader_itinerary = leader.itinerary
    leader_positions = [leader.xcor]
    leader_speeds = [leader.v_x] # ここは計算しないといけない！！
    print(f"target_time: {target_time}")
    if ttc < 1:
        print("ttc:", ttc)
        raise ValueError("ttc is too small")

    # シミュレーションのパラメータ
    total_time = target_time - current_time
    time_step = 0.5
    steps = int(total_time / time_step)
    time_array = np.arange(current_time + time_step, target_time + time_step, time_step)
    # 続いてleader_potisionsとleader_acc_itineraryを元にleaderの位置と速度を計算する
    for time in time_array:
        leader_acc = get_acc_for_time(leader_acc_itinerary, time) # ここ、加速度の時間幅の違いによってはバグの温床になるので注意！
        leader_speed = leader_speeds[-1]
        leader_speeds.append(leader_speed + leader_acc * time_step)
        leader_positions.append(leader_positions[-1] + leader_speed * time_step + 0.5 * leader_acc * time_step**2)
        # この時点でのleaderの情報を表示（その時間で動画を止めた時の位置などが出力される）
        # print(current_time +time, leader_acc, leader_speeds[-1], leader_positions[-1])
  
    # 後続車のパラメータ
    follower_acc = follower_acc_solver(follower, leader_positions, time_step, time_array,ttc, leader_speeds)
    print(leader_acc_itinerary)
    print(follower_acc)
    return follower_acc, time_step, steps

def follower_acc_solver(follower, leader_positions, time_step, time_array, ttc, leader_speeds):
    """
    follower: 後続車のオブジェクト
    leader_positions: 先頭車の位置のリスト
    time_step: 時間ステップ
    time_array: 時間のリスト
    ttc: Time to Collision (衝突までの時間)
    follower_acc: 返り値となる後続車の加速度のリスト.
    """

    follower_acc = [0 for _ in range(len(time_array))]  # 後続車の加速度のリスト
    for count in range(len(time_array)): # 何番目の加速度をいじるか. 
        follower_acc = [0 for _ in range(len(time_array))]  # 後続車の加速度のリスト
        safe_distance_met = True
        if count > 0:
            for idx in range(count):
                follower_acc[idx] = -2
        follower_positions = [follower.xcor]  # 後続車の位置のリスト
        follower_speeds = [follower.v_x]  # 後続車の速度のリスト


        for i, time in enumerate(time_array):
            # 先頭車と後続車の間の距離
            distance = leader_positions[i] - follower_positions[i] # ここはあえてindexを合わせておく. 
            follower_speed = follower_speeds[-1]

            # 安全距離 (先頭車の速度にTTCを掛けた値)
            safe_distance = follower_speed * ttc
            if distance < safe_distance: # 安全距離よりも近い場合
                # print(f"count={count}, i={i},time={time},distance={distance}, follower_pos={follower_positions[-1]}, ttc={distance/follower_speed}")
                safe_distance_met = False
                break
                 
            next_speed = follower_speed + follower_acc[i] * time_step
            follower_speeds.append(next_speed)
            next_position = follower_positions[i] + follower_speed * time_step + 0.5 * follower_acc[i] * time_step**2
            follower_positions.append(next_position)

        if safe_distance_met:
            # print(f"count={count}, i={i},time={time},distance={distance}, follower_pos={follower_positions[-1]}, ttc={distance/follower_speed}")
            break
    
    # ここまでで最低限の加速度を担保
    # ここからは最大の加速度を保証していく. 

    solution = []
    detail_result = []
    
    for count in range(len(time_array)): # 何番目の加速度をいじるか. 
        detailList = []
        is_safety_distance_met = True
        if count > 0:
            for idx in range(count):
                follower_acc[len(follower_acc)- 1 - idx] = 2
        follower_positions = [follower.xcor]  # 後続車の位置のリスト
        follower_speeds = [follower.v_x]  # 後続車の速度のリスト
        for i, time in enumerate(time_array):
            # 先頭車と後続車の間の距離
            distance = leader_positions[i] - follower_positions[i] # ここはあえてindexを合わせておく. 
            follower_speed = follower_speeds[-1]

            # 安全距離 (先頭車の速度にTTCを掛けた値)
            safe_distance = follower_speed * ttc
            if distance < safe_distance: # 安全距離よりも近い場合加速しすぎなのでbreak. 
                is_safety_distance_met = False
                # print(f"count={count}, i={i},time={time},distance={distance}, follower_pos={follower_positions[-1]}, ttc={distance/follower_speed}")
                break
                 
            next_speed = follower_speed + follower_acc[i] * time_step
            follower_speeds.append(next_speed)
            next_position = follower_positions[i] + follower_speed * time_step + 0.5 * follower_acc[i] * time_step**2
            follower_positions.append(next_position)
            detailList.append({"time":time, "distance":distance, "follower_pos":follower_positions[-1], "leader_pos":leader_positions[i], "follower_speed":follower_speed, "ttc":distance/follower_speed})
            
        if not is_safety_distance_met:
            print("加速上限到達")  
            print(f"count={count}, i={i},time={time},distance={distance}, follower_pos={follower_positions[-1]}, leaderPos={leader_positions[i]}, follower_speed={follower_speed}, ttc={distance/follower_speed}")
            break
        else:
            # print("sol 更新")
            solution = follower_acc.copy()
            detail_result = detailList.copy()

            
    print("===RESULT====")
    print("sol: ",solution)
    print("acc: ",follower_acc)    
    print("===DETAIL====")
    print(detail_result)
    print("===========")

    return solution
    
    
    
    

    
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


