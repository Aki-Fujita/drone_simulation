from .simple_funcs import update_acc_itinerary, will_collide
from .optimizer_for_follower import update_acc_itinerary_with_accel, should_brake, can_reach_after_designated_eta, crt_acc_itinerary_for_decel_area, follower_acc_solver, merge_acc_itinerary
import copy
import pandas as pd


def calc_late_avoid_with_early_avoid_leader(**kwargs):
    follower = kwargs.get("follower")
    ttc = kwargs.get("ttc", 0)
    noise_x = kwargs.get("xe")  # noiseの発生位置
    noise_t = kwargs.get("te")  # noiseを通過する時刻（ノイズ終了時刻くらい）
    current_time = kwargs.get("current_time", [])
    eta_of_leader = kwargs.get("eta_of_leader", {})
    leader_finish_time = eta_of_leader.loc[eta_of_leader["eta"].idxmax(
    )]["eta"]
    if ttc < 1:
        print("ttc:", ttc)
        raise ValueError("ttc is too small")

    # シミュレーションのパラメータ
    total_time = leader_finish_time - current_time
    time_step = 0.5
    steps = int(total_time / time_step)

    # リーダーのETAとノイズの通過時刻を元に最速のETAを計算する.
    constraints = create_earliest_etas(
        eta_of_leader, noise_x, noise_t, ttc, follower.my_etas)
    print(f"constraints: {constraints}")

    # 後続車のパラメータ
    acc_itinerary = follower_acc_solver(
        follower, constraints, ttc, current_time)
    merged_acc_itinerary = merge_acc_itinerary(
        pre_itinerary=follower.acc_itinerary, new_itinerary=acc_itinerary)
    print("L34: merged:", merged_acc_itinerary)
    return merged_acc_itinerary, time_step, steps


def create_earliest_etas(leader_eta, noise_x, noise_t, ttc, my_etas):
    constraints = []
    print("leader_eta: ", leader_eta)
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
    print("constraints: ", constraints)
    return pd.DataFrame(constraints)


def _calc_late_avoid_with_early_avoid_leader(**kwargs):
    """
    これは自分の前のleaderがnoiseを早避けしていて、リーダーはいるがノイズ前では気にしなくて良い場合の処理
    """
    follower = kwargs.get("follower")
    xe = kwargs.get("xe")  # noiseの発生位置
    te = kwargs.get("te")  # noiseを通過する時刻（ノイズ終了時刻くらい）
    current_time = kwargs.get("current_time")
    eta_of_leader = kwargs.get("eta_of_leader")
    ttc = kwargs.get("ttc")
    car_params = {"decel": follower.a_min, "accel": follower.a_max}
    acc_itinerary = follower.acc_itinerary

    earliest_etas = create_earliest_etas(eta_of_leader, ttc)
    noise_avoid_point = {"xe": xe, "te": te}
    initial_params = {"v0": follower.v_x,
                      "x0": follower.xcor, "t0": current_time}
    start_params = copy.deepcopy(initial_params)  # 各区間をスタートするときのパラメータ
    print("leader_eta: ", eta_of_leader, follower.xcor)
    print("earliest_etas: ", earliest_etas)
    itinerary_from_now = [{"t_start": current_time, "acc": 0,
                           "v_0": follower.v_x, "t_end": earliest_etas[0]["eta"]}]

    for idx, earliest_eta in enumerate(earliest_etas):
        if follower.xcor >= earliest_eta["x"]:
            print("skipped: ", earliest_eta, follower.xcor)
            continue
            print()
        # print("====now testing: ", earliest_eta, "====")
        # print("===start_params: ", start_params, "===")
        # print("===acc_itinerary: ", acc_itinerary, "===")
        eta_boundary = {"xe": earliest_eta["x"], "te": earliest_eta["eta"]}
        x_start = earliest_eta["x"]
        next_goal_x = earliest_etas[idx +
                                    1]["x"] if idx < len(earliest_etas)-1 else None
        should_brake_for_next_interval = will_collide(
            **start_params, **eta_boundary, decel=car_params["decel"])
        # noiseより手前のwaypointに対する処理.
        if earliest_eta["x"] < noise_avoid_point["xe"]:
            # ブレーキを踏む必要がない場合
            if not should_brake(**start_params, **eta_boundary) and not should_brake_for_next_interval:
                upcoming_wps = [{"xe": e["x"], "te": e["eta"]}
                                for i, e in enumerate(earliest_etas) if i >= idx]
                if all([not should_brake(**start_params, **eta_boundary) for eta_boundary in upcoming_wps]):
                    new_itinerary, sp = update_acc_itinerary_with_accel(itinerary_from_now, start_params, upcoming_wps, car_params={
                                                                        **car_params, "acc": 2, "v_max": follower.v_max}, current_x=follower.xcor)
                    itinerary_from_now = new_itinerary
                    start_params = sp
                else:
                    continue
                continue
            if can_reach_after_designated_eta(**start_params, **eta_boundary, car_params=car_params):
                a, eta = crt_acc_itinerary_for_decel_area(**start_params, **eta_boundary, ve=None, car_params=car_params,
                                                          step_size=0.5, earliest_etas=earliest_etas)
                v = a[-1]["v_0"]
                start_params = {"v0": v, "x0": eta_boundary["xe"], "t0": eta}
                itinerary_from_now = update_acc_itinerary(
                    itinerary_from_now, a)
                continue
            else:
                raise ValueError("絶対に当たってしまうノイズがあります。L65")

        # noiseより後のwaypointに対する処理
        acc_itinerary = update_acc_itinerary(
            acc_itinerary, itinerary_from_now)  # まずはここまでのitineraryをまとめる.

    return
