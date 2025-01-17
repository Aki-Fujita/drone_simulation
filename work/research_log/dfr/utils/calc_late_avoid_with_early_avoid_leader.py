from .optimizer_for_follower import follower_acc_solver, merge_acc_itinerary
import pandas as pd
import logging
logging.basicConfig(level=logging.INFO)  # INFOレベル以上を表示


def calc_late_avoid_with_early_avoid_leader(**kwargs):
    follower = kwargs.get("follower")
    ttc = kwargs.get("ttc", 0)
    noise_x = kwargs.get("xe")  # noiseの発生位置
    noise_t = kwargs.get("te")  # noiseを通過する時刻（ノイズ終了時刻くらい）
    current_time = kwargs.get("current_time", [])
    eta_of_leader = kwargs.get("eta_of_leader", {})
    leader = kwargs.get("leader", {})
    leader_finish_time = eta_of_leader["eta"].max()
    leader_finish_time = float(leader_finish_time) 
    if ttc < 1:
        raise ValueError("ttc is too small")

    # シミュレーションのパラメータ
    total_time = leader_finish_time - current_time
    time_step = 0.5
    steps = int(total_time / time_step)

    # リーダーのETAとノイズの通過時刻を元に最速のETAを計算する.
    constraints = create_earliest_etas(
        eta_of_leader, noise_x, noise_t, ttc, follower.my_etas)
    

    # 後続車のパラメータ
    acc_itinerary = follower_acc_solver(
        follower, constraints, ttc, current_time, leader)
    # if follower.car_idx == 39:
    #     print("L25: constraints: ", constraints, "\n", "acc_itinerary: ", acc_itinerary)
    merged_acc_itinerary = merge_acc_itinerary(
        pre_itinerary=follower.acc_itinerary, new_itinerary=acc_itinerary)
    logging.debug("L34: merged:", merged_acc_itinerary)
    return merged_acc_itinerary, time_step, steps


def create_earliest_etas(leader_eta, noise_x, noise_t, ttc, my_etas):
    constraints = []
    logging.debug("leader_eta: ", leader_eta)
    for my_eta in (my_etas):
        x_coord = my_eta['x']
        if x_coord < noise_x:
            # (a) my_etaのx座標がnoise_xより手前の場合
            constraints.append({**my_eta, "eta": my_eta['eta'] - ttc - 0.15})

        elif x_coord == noise_x:
            # (b) my_etaのx座標がnoise_xと等しい場合
            constraint = {
                'eta': noise_t - ttc,
                'car_idx': my_eta['car_idx'],
                'type': 'noise',
                'x': noise_x
            }
            constraints.append(constraint)

        else:
            # (c) my_etaのx座標がnoise_xより後ろの場合
            corresponding_leader_eta = leader_eta[leader_eta['x'] == x_coord]
            if not corresponding_leader_eta.empty:
                adjusted_eta = corresponding_leader_eta.iloc[0]['eta']
                my_eta['eta'] = adjusted_eta
            constraints.append(my_eta)

    # constraintsをデータフレームに変換してreturn
    logging.debug("constraints: ", constraints)
    return pd.DataFrame(constraints)
