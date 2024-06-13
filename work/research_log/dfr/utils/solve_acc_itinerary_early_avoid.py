import pandas as pd
import copy
from .calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary


def solve_acc_itinerary_early_avoid(**kwargs):
    """
    Output: acc_itinerary if possible else False
    ノイズを早避けするためのacc_itineraryを求める. 
    20240604時点ではナイーブな実装でいく.
    【方針】
    if 加速できる: # ここでの判定基準は、ノイズの1つ手前までのETAに当たらないこと
        => 加速する. 
    else: 速度をキープ
    """
    print("solve_acc_itinerary_early_avoid")
    time_step = 0.5
    noise_start_time = kwargs.get("noise_start_time", None)
    car = kwargs.get("car", None)
    current_time = kwargs.get("current_time", None)
    leader_eta = kwargs.get("leader_eta", None)
    ttc = kwargs.get("ttc", None)
    xe = kwargs.get("xe", None)
    waypoints = kwargs.get("waypoints", None)
    acc_itinerary = car.acc_itinerary
    if isinstance(leader_eta, pd.DataFrame) and leader_eta.empty:
        # 先頭のクルマがいない場合
        leader_eta = None
        acc_itinerary = calc_leader_acc_itinerary(car, current_time)
        return acc_itinerary
    earliest_etas = create_earliest_etas(leader_eta, ttc, waypoints)
    upper_constraint_list = [{"xe": xe, "te": noise_start_time}]
    upper_constraint = upper_constraint_list[0]

    acc_itinerary = car.acc_itinerary
    acc_itinerary_from_now = []

    """
    ここから計算.
    求めるのは、加速度のitinerary（どの加速度をどのくらい出すかの情報）
    ノイズより手前の場合 => 前のETAリストに近づけるだけ近づく.
    """
    initial_params = {"v0": car.v_x, "x0": car.xcor, "t0": current_time}
    start_params = copy.deepcopy(initial_params)  # 各区間をスタートするときのパラメータ
    for idx, fastest_eta in enumerate(earliest_etas):
        print()
        print("====now testing: ", fastest_eta, "====")
        print("===start_params: ", start_params, "===")
        print("===acc_itinerary: ", acc_itinerary, "===")
        x_start = fastest_eta["x"]
        next_goal_x = earliest_etas[idx +
                                    1]["x"] if idx < len(earliest_etas)-1 else None

        if fastest_eta["x"] < upper_constraint["xe"]:
            # スタートより後でかつnoiseより手前のwaypointに対する処理.
            # この場合は可能な範囲で頑張って加速する.
            arrival_time_if_cruise = (
                # 等速で走った時の到着時刻
                x_start - start_params["x0"]) / start_params["v0"] + start_params["t0"]
            if arrival_time_if_cruise > fastest_eta["eta"]:
                # 加速できる場合は加速できるかぎりする.
                arrival_time = arrival_time_if_cruise
                count = 0
                next_start_params = copy.copy(start_params)

                while arrival_time > fastest_eta["eta"]:
                    # print("early avoid loop. Count", count)
                    count += 1
                    if count > 100:
                        raise ValueError("Something wrong")
                    acc_period_length = count * time_step
                    acc_period = {"t_start": start_params["t0"], "acc": car.a_max,
                                  "v_0": start_params["v0"], "t_end": start_params["t0"]+acc_period_length}
                    acc_period_end = x_start + 0.5 * car.a_max * \
                        acc_period_length**2 + \
                        start_params["v0"] * acc_period_length
                    v_after_acc = start_params["v0"] + \
                        car.a_max * acc_period_length
                    eta_of_next_goal = (
                        next_goal_x - x_start)/v_after_acc + start_params["t0"] if next_goal_x else 1e7
                    cruise_after_accel = {
                        "t_start": start_params["t0"]+acc_period_length, "acc": 0, "v_0": v_after_acc, "t_end": eta_of_next_goal}
                    cruise_params = {"v0": v_after_acc,
                                     "x0": next_goal_x, "t0": eta_of_next_goal}

                    """
                    ループを抜ける条件
                    ・加速終了時の速度がv_maxを超える
                    ・加速区間終了時が次のWPを超えてしまう. 
                    ・加速しすぎてノイズより手前のWPまでのいずれかでブレーキが必要になる. 
                    """
                    if v_after_acc > car.v_max or acc_period_end > next_goal_x:
                        print("ループ抜ける判定基準1:", v_after_acc,
                              car.v_max, acc_period_end, next_goal_x)
                        start_params = next_start_params
                        break

                    # どこかで引っかかったらという条件.
                    upcoming_wps_before_noise = [{"xe": e["x"], "te": e["eta"]} for i, e in enumerate(
                        earliest_etas) if i >= idx and e["x"] <= upper_constraint["xe"]]
                    if not all([not should_brake(**cruise_params, **earliest_eta) for earliest_eta in upcoming_wps_before_noise]):
                        print("ループ抜ける判定基準2:", [should_brake(
                            **cruise_params, **earliest_eta) for earliest_eta in upcoming_wps_before_noise])
                        print("detail: ", [{"eta": earliest_eta["te"], "arrival_time": (earliest_eta["xe"] - cruise_params["x0"]) /
                              cruise_params["v0"]+cruise_params["t0"], "x": earliest_eta["xe"]} for earliest_eta in upcoming_wps_before_noise])
                        start_params = next_start_params
                        break

                    # 上の条件に該当しなかった場合はまだ加速して良いということなので、今の結果を保存しループを続ける.
                    _acc_itinerary = [acc_period, cruise_after_accel]
                    # print(_acc_itinerary)
                    acc_itinerary = update_acc_itinerary(
                        acc_itinerary, _acc_itinerary)

                    # 次のループのための初期値（スタート時のパラメータ）を更新
                    next_start_params = {"v0": v_after_acc,
                                         "x0": next_goal_x, "t0": eta_of_next_goal}

        elif fastest_eta["x"] >= upper_constraint["xe"]:  # ノイズより後ろのWPsが来た場合.
            # まずはacc_itineraryからノイズの到着時間を計算
            x_at_noise = calc_distance_from_acc_itinerary(
                acc_itinerary, upper_constraint["te"])
            if x_at_noise < upper_constraint["xe"]:
                print("最速で行ってもノイズに当たってしまうので早避け不可能")
                print("detail:", x_at_noise,
                      upper_constraint["xe"], upper_constraint["te"])

                return False

            else:
                # これは早避けが可能な場合.
                # しっかり減速するための経路設計をすることもできるが一旦acc_itineraryを返しておく
                return acc_itinerary

    return acc_itinerary


def create_earliest_etas(leader_eta, ttc, waypoints):
    leader_eta_list = leader_eta.sort_values(
        by=["x"]).to_dict(orient="records")
    earliest_etas = [{"x": leader_eta["x"], "eta": leader_eta["eta"]+ttc}
                     for leader_eta in leader_eta_list]
    return earliest_etas


"""
先頭車がいない場合はとりあえず最大速度まで加速してそのまま走る. 
"""


def calc_leader_acc_itinerary(car, current_time):
    first_acc_period = (car.v_max - car.v_x) / car.a_max
    acc_itinerary = [{"t_start": current_time, "acc": car.a_max,
                      "v_0": car.v_x, "t_end": current_time + first_acc_period}]
    cruise_period = {"t_start": current_time + first_acc_period,
                     "acc": 0, "v_0": car.v_max, "t_end": 1e7}
    acc_itinerary.append(cruise_period)
    return acc_itinerary


def should_brake(v0, x0, t0, xe, te):
    arrival_time = (xe - x0) / v0 + t0
    # print(f"Arrival Time: {arrival_time}, te: {te}, v0:{v0}, x0:{x0}, t0:{t0}")
    if arrival_time > te:  # 普通に等速で進んでもETAより後ろになる場合はブレーキをかけないで良い.
        return False
    return True


def update_acc_itinerary(current_itinerary, new_itinerary):
    """
    itineraryのappend処理をする
    加速度が同じだったら区間を繋げる
    """
    # print("acc_itinerary updates:",current_itinerary, new_itinerary)
    result = copy.deepcopy(current_itinerary)
    # そもそもnew_itineraryの方が前から始まっていたらnew_itineraryで完全にreplaceする.
    if new_itinerary[0]["t_start"] == current_itinerary[0]["t_start"]:
        return new_itinerary
    # 末尾と先頭の加速度が等しい場合は区間を繋げる
    if result[-1]["acc"] == new_itinerary[0]["acc"]:
        result[-1]["t_end"] = new_itinerary[0]["t_end"]
        return result

    # 新しいものが中間に挿入される場合.
    prev_start = result[0]["t_start"]
    prev_end = result[-1]["t_end"]
    res = []
    if new_itinerary[0]["t_start"] < prev_end:
        for accObj in current_itinerary:
            if accObj["t_end"] < new_itinerary[0]["t_start"]:
                res.append(accObj)
            else:
                accObj["t_end"] = new_itinerary[0]["t_start"]
                res.append(accObj)
                break
        return res + new_itinerary

    # 基本的にはこれ. 昔のものの末尾に新しいものを追加する.
    result = current_itinerary + new_itinerary
    return result
