from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
import math
from calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary

"""
メモ: 全車の最大減速度は揃えた方が良い（そうでないと破綻するルールもある）
"""


"""
加速度のログからvとxの時系列を計算する
"""
def calc_distance_from_a(a_series, x0, v0, dt, N):
    print(a_series)
    v = np.zeros(N+1)
    x = np.zeros(N+1)
    v[0] = v0
    x[0] = x0

    for i in range(1, N+1):
        v[i] = v[i-1] + a_series[i-1] * dt
        x[i] = x[i-1] + v[i-1] * dt + 0.5 * a_series[i-1] * dt**2
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
前の車もnoiseを late_avoidしている前提で作られている！
"""
def optimizer_for_follower(**kwargs):
    follower=kwargs.get("follower")
    xe = kwargs.get("xe", 0) # noise発生位置
    noise_pass_time = kwargs.get("noise_pass_time", 0) # ノイズに到達する時間(しかしノイズ以後も最適化対象領域なので不要かも). 
    ttc = kwargs.get("ttc", 0)
    leader = kwargs.get("leader", 0) # 先行車のオブジェクト
    current_time = kwargs.get("current_time", [])
    leader_acc_itinerary = leader.acc_itinerary 
    leader_positions = [leader.xcor]
    leader_speeds = [leader.v_x] # ここは計算しないといけない！！
    eta_of_leader = kwargs.get("eta_of_leader", {})
    leader_finish_time = eta_of_leader.loc[eta_of_leader["eta"].idxmax()]["eta"]
    print(f"noise_pass_time: {noise_pass_time}, leader_finish_time: {leader_finish_time}")
    if ttc < 1:
        print("ttc:", ttc)
        raise ValueError("ttc is too small")

    # シミュレーションのパラメータ
    total_time = leader_finish_time - current_time
    time_step = 0.5
    steps = int(total_time / time_step)
    time_array = np.arange(current_time, leader_finish_time + time_step, time_step)
    print(f"XE: {xe}")
    # 続いてleader_potisionsとleader_acc_itineraryを元にleaderの位置と速度を計算する
    for time in time_array:
        leader_acc = get_acc_for_time(leader_acc_itinerary, time) # ここ、加速度の時間幅の違いによってはバグの温床になるので注意！
        leader_speed = leader_speeds[-1]
        leader_speeds.append(leader_speed + leader_acc * time_step)
        leader_positions.append(leader_positions[-1] + leader_speed * time_step + 0.5 * leader_acc * time_step**2)
        # この時点でのleaderの情報を表示（その時間で動画を止めた時の位置などが出力される）
        # print(current_time +time, leader_acc, leader_speeds[-1], leader_positions[-1])
  
    # 後続車のパラメータ
    print(f"currentTime: {current_time}, leader_pos:{leader_positions[0]}, leader_speed:{leader_speeds[0]}")
    follower_acc = follower_acc_solver(follower, eta_of_leader, ttc)
    print(leader_acc_itinerary)
    print(follower_acc)
    return follower_acc, time_step, steps

def follower_acc_solver(follower, eta_of_leader, TTC):
    """
    基本方針
    (a) まずleaderのETAを一通り確認し、それに対し+TTC秒以上開けたETAを作成する
    (b) 続いてそのETAがvalidであるかを検証し、その上で加速度のitineraryを作成する
    """
    print("LEADER ETA: \n", eta_of_leader)

    # (a) まずはLeaderのETAから自分に可能な最速のETAを計算する（前準備）
    earliest_etas = []
    for eta_info in eta_of_leader:
        earilist_eta = eta_info["eta"] + TTC
        earliest_etas.append({**eta_info, "eta":earilist_eta, "car_idx": follower.car_idx})
    
    # (b) 続いて上のETAがvalidであるかを検証し、その上で加速度のitineraryを作成する
    for eta_plan in earliest_etas:
        
        return

"""
よく使う基本関数
初期条件{v0, x0, t0}と目標条件{ve, xe, te}から, 自分がブレーキすべきかどうかを判断する
initial_paramsのままゴールまで進んだ時に到着時間がt_eを超えるかどうかを計算する
"""      
def should_brake(v0, x0, t0, xe, te):
    arrival_time = (xe - x0) / v0 + t0
    if arrival_time > te: # 普通に等速で進んでもETAより後ろになる場合はブレーキをかけないで良い.
        return False
    return True

"""
should_brakeと合わせて利用する. 
初期条件{v0, x0, t0}と目標条件{ve, xe, te}から、最短到達時間を計算する
返り値: te, acc_itinerary
"""
def calc_valid_eta_with_brake(v0, x0, t0, xe, te, car_params):
    if not should_brake(v0, x0, t0, xe, te):
        return te, [{"t_start":t0, "acc":0, "v_0":v0}]
    # ブレーキの必要がある場合
    decel = car_params.get("decel",None) # 減速度には正の値を入れる. 
    accel = car_params.get("accel",None)
    # ここでacc_itineraryを作成する
    time_step = 0.5
    time_array = []
    acc_array = []
    count = 0
    v = v0
    initial_conditions = {"v0":v0, "x0":x0, "t0":t0}
    while x <=xe or count < 100:
        # まず、今のスピードで等速に動けば間に合う場合はそのスピードを維持する. 
        acc_array.append(decel)
        cover_distance = v * time_step - 0.5 * decel * time_step**2
        v -= decel * time_step

        count += 1

"""
ある場所に、定められた時間以後に到達できるかを判定する. 
=> 要するにこれは、delta_t秒でなるべく距離を出さないようにして、その結果delta_x[m]進まなければクリア
=> いずれにせよ車の最善策は全力でブレーキ
(1) delta_t <= brake_timeの場合: これは速度が0にならない場合: 
    => cover_distance < delta_xならクリア
(2) delta_t > brake_timeの場合: これは速度が0になる場合
    => delta_x > brake_distanceならクリア
"""
def can_reach_after_designated_eta(v0, x0, t0, xe, te, car_params):
    if not should_brake(v0, x0, t0, xe, te):
        return True
    """
    逆に今からずっと急ブレーキをしてxe, teの前に止まれるかを判定する.
    """
    delta_t = te - t0
    delta_x = xe - x0
    decel = -1 * abs(car_params.get("decel",None)) # 減速度には負の値を入れる.
    brake_time = v0 / abs(decel)
    brake_distance = (v0**2)/(2*abs(decel))  #速度が0になるまでの時間
    if delta_t <= brake_time: # 速度が0にならない場合
        cover_distance = v0 * delta_t + 0.5 * decel * delta_t**2
        if cover_distance < delta_x:
            return True
        else:
            return False # 全力でブレーキをかけてもdelta_t秒後にはゴールについてしまう. 
    else: # 速度を0にできる場合
        if brake_distance < delta_x:
            return True
        else:
            return False

"""
2つのwaypoint間の進み方を決めるための関数.
can_reach_after_designated_etaがTrueだった場合に、その条件下での最適な加速度を計算する.

"""
def crt_acc_itinerary(v0, x0, t0, ve, xe, te, car_params, step_size):
    decel = abs(car_params.get("decel",None))*-1 
    steps = int((te - t0) / step_size) + 1 # 始点から終点までの秒数
    cover_distance = 1e8
    loop_count = 0
    while cover_distance > xe - x0 or loop_count < steps:
        print("Loops: ",loop_count)
        loop_count += 1
        # まずは加速度を変更してみる
        decel_period = loop_count * step_size
        acc_itinerary = [{"t_start":t0, "acc":decel, "v_0":v0, "t_end":t0+decel_period}]
        acc_itinerary.append({"t_start":t0+ decel_period, "acc":0, "v_0":v0 + decel * decel_period, "t_end":te})
        # acc_itineraryからt=teでの位置を計算する
        print(acc_itinerary)
        cover_distance = calc_distance_from_acc_itinerary(acc_itinerary, te)
        if cover_distance < xe - x0:
            return acc_itinerary
        
    return False

# こいつがノイズに早く着きすぎないように修正する必要あり. 
def _follower_acc_solver(follower, ):
    """
    follower: 後続車のオブジェクト
    leader_positions: 先頭車の位置のリスト
    time_step: 時間ステップ
    time_array: 時間のリスト
    ttc: Time to Collision (衝突までの時間)
    follower_acc: 返り値となる後続車の加速度のリスト.
    """

    follower_acc = [0 for _ in range(len(time_array))]  # 後続車の加速度のリスト
    print(f"leader_speeds, {leader_speeds}")
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
            min_distance = follower_speed * ttc * 0.9 # TTCより近づいてしまうのはやむを得ないがこれ以上はまずい、という距離
            if distance < safe_distance: # 安全距離よりも近い場合
                if distance < min_distance:
                    safe_distance_met = False
                    print(f"failed:t={time}, d={distance}, TTC={distance/follower_speed}")
                    # print(f"f_pos_log: {follower_positions}")
                    print("failed because of min-TTC violtaion.")
                    break
                # ここでRSS距離を交えた計算をする
                if not isRssOK(distance, leader_speeds[i], follower_speed):
                    # print(follower_acc)
                    print("failed because of RSS violtaion.")
                    print("detail: ",count, i, time, distance, leader_positions[i], follower_positions[i], follower_speed, ttc)
                    safe_distance_met = False
                    break
                 
            next_speed = follower_speed + follower_acc[i] * time_step
            follower_speeds.append(next_speed)
            next_position = follower_positions[i] + follower_speed * time_step + 0.5 * follower_acc[i] * time_step**2
            follower_positions.append(next_position)

        # time_array内のすべての時間について先頭車との距離が安全距離以上である場合.
        if safe_distance_met:
            follower_speed = follower_speeds[-1]
            print(f"last check: {distance}")
            if isRssOK(distance, leader_speeds[i], follower_speed):
                print(f"OK: f_pos: {follower_positions[-1]}, l_pos:{leader_positions[-1]}, \n f_speeds:{follower_speeds}")
                break
    
    # ここまでで最低限の加速度を担保
    # ここからは最大の加速度を保証していく
    print(f"Solution of first loop: \n {follower_acc}")
    print("=======Start Second Loop=======")  

    solution = []
    detail_result = []
    is_safety_distance_met = True
    distance_array = [leader_positions[i] - follower_positions[i] for i in range(len(time_array))] # 車間距離の時系列
    
    """
    加速側のアルゴリズム（制約充足アルゴリズム）
    << 方針 >>
    0. 前処理: distance_arrayの差分系列を作り、加速すべきポイントを列挙する
    1(a). distance_arrayを見ながら車間距離が空いたら加速し、詰まったら減速する
    1(b). と思ったがdistance_arrayではなくleader_accを見ながら加速したほうがdelayがないかもしれない.
    1(c). ヘリーモデルの要領で、車間距離の変化を見ながら加速度を調整する方針が良いかも。← 一旦これでいく

    ## #以下の条件を満たさなくなったらbreak# 
    ・前の車との車間距離が近くなりすぎる
    ・ゴールについてしまう（でも車間距離がある時点でこれはないはず） 
    """
    print(f"distance_array: {distance_array}")

    """
    まずは前処理で差分系列を作成
    """
    diff_distance_array = [0] # 初項は0で初期化. 車間距離の変化を記録
    accelerate_moment = []
    for i in range(1, len(distance_array)):
        diff = distance_array[i] - distance_array[i-1]
        diff_distance_array.append(diff)
        if diff > 0:
            accelerate_moment.append(i)
    can_accel_point = min(accelerate_moment) # これが最初に加速すべきポイント（前の車との車間が開くタイミング）
    print(f"can_accel_point: t={can_accel_point*time_step}, d={follower_positions[can_accel_point]}")

    """
    続いて加速度の最適化. 一旦noise_pass_timeは考えないことにする. 
    基本方針としては、車間距離から前の車の速度を推定し、それに合わせて自分の加速度を調整する.
    """

    result_of_first_loop = follower_acc.copy()
    follower_positions = [follower.xcor]  # 後続車の位置のリスト
    follower_speeds = [follower.v_x]  # 後続車の速度のリスト
    
    for i, time in enumerate(time_array): # count変数は加速の回数

        if i < can_accel_point: # すでに加速度が定まっている場合
            acc = result_of_first_loop[i]
        else: # 一つ目のループで加速度が定まっていないとき
            """
            この場合はETAの点を最初に決めてそれが実行可能かを事後的に判断する.
            """
        follower_acc[i] = acc
        follower_speed = follower_speeds[-1]
        next_speed = follower_speed + acc * time_step
        follower_speeds.append(next_speed)
        cover_distance = follower_speed * time_step + 0.5 * follower_acc[i] * time_step**2
        # print(f"f_pos: {follower_positions[-1]}, f_speed: {follower_speeds[-1]}, cover_distance: {cover_distance}, acc:{acc}")
        
        next_position = follower_positions[i] + follower_speed * time_step + 0.5 * follower_acc[i] * time_step**2
        follower_positions.append(next_position)   
        solution = follower_acc.copy()
            
    print("===RESULT====")
    print("sol: ",solution)
    print("acc: ",follower_acc) 
    print("f_pos: ",follower_positions) 
    print("f_speed: ",follower_speeds) 
    print("===DETAIL====")
    distance_array = [leader_positions[i] - follower_positions[i] for i in range(len(time_array))] # 車間距離の時系列
    print(distance_array)
    print("===========")

    return solution

def isRssOK(distance, leader_speed, follower_speed):
    leader_blake_max = 3
    follower_blake_max = 3
    leader_blake_distance = leader_speed ** 2 / (2 * leader_blake_max) # ここは一旦ブレーキの最大加速度を3としている
    follower_blake_distance = follower_speed ** 2 / (2 * follower_blake_max)
    if distance + leader_blake_distance < follower_blake_distance:
        print("===FAIL DETAIL===")
        print(distance, leader_blake_distance, follower_blake_distance, "leader speed:", leader_speed, "follower speed:", follower_speed)
        print("=================")
    return distance + leader_blake_distance > follower_blake_distance

def is_every_eta_valid(follower_acc, eta_of_leader, ttc):
    return True 

def conduct_tests():
    te = 20
    print("===TEST START=====")
    a = crt_acc_itinerary(**{"v0":20, "x0":0, "t0":8, "ve":40, "xe":120, "te":te, "car_params":{"decel":2}, "step_size":0.5})
    print("===TEST END=====")
    print(calc_distance_from_acc_itinerary(a, te))
    print(f"result:{a}")

    
if __name__ == "__main__":
    conduct_tests()