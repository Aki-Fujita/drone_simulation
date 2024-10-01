import pandas as pd
import copy
from .calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary
from .optimizer_for_follower import can_reach_after_designated_eta
from .will_collide import will_collide
from .crt_acc_itinerary_for_decel_area import crt_acc_itinerary_for_decel_area
from .simple_funcs import create_earliest_etas


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
        acc_itinerary_from_now = calc_leader_acc_itinerary(car, current_time)
        acc_itinerary = update_acc_itinerary(
            car.acc_itinerary, acc_itinerary_from_now)
        return acc_itinerary

    # ここからは前の車がいる場合のearly avoidの処理.
    earliest_etas = create_earliest_etas(leader_eta, ttc)
    noise_avoid_point = {"xe": xe, "te": noise_start_time}

    acc_itinerary = car.acc_itinerary

    """
    ここから計算.
    求めるのは、加速度のitinerary（どの加速度をどのくらい出すかの情報）
    ノイズより手前の場合 => 前のETAリストに近づけるだけ近づく.
    """
    initial_params = {"v0": car.v_x, "x0": car.xcor, "t0": current_time}
    start_params = copy.deepcopy(initial_params)  # 各区間をスタートするときのパラメータ
    for idx, fastest_eta in enumerate(earliest_etas):
        if car.xcor >= fastest_eta["x"]:
            print("skipped: ", fastest_eta, car.xcor)
            continue
        print()
        print("====now testing: ", fastest_eta, "====")
        # print("===start_params: ", start_params, "===")
        # print("===acc_itinerary: ", acc_itinerary, "===")
        x_start = fastest_eta["x"]
        next_goal_x = earliest_etas[idx +
                                    1]["x"] if idx < len(earliest_etas)-1 else None
        # noiseより手前のwaypointに対する処理.
        if fastest_eta["x"] < noise_avoid_point["xe"]:
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
                    if count > 300:
                        raise ValueError("Something wrong")
                    acc_period_length = count * time_step
                    acc_period = {"t_start": start_params["t0"], "acc": car.a_max, "x_start": start_params["x0"],
                                  "v_0": start_params["v0"], "t_end": start_params["t0"]+acc_period_length}
                    print(acc_period, x_start, next_goal_x)
                    acc_period_end = x_start + 0.5 * car.a_max * \
                        acc_period_length**2 + \
                        start_params["v0"] * acc_period_length
                    v_after_acc = start_params["v0"] + \
                        car.a_max * acc_period_length
                    eta_of_next_goal = (
                        next_goal_x - acc_period_end)/v_after_acc + start_params["t0"] + acc_period_length if next_goal_x else 1e7
                    cruise_after_accel = {
                        "t_start": start_params["t0"]+acc_period_length, "acc": 0, "v_0": v_after_acc, "t_end": eta_of_next_goal, "x_start": acc_period_end}
                    cruise_params = {"v0": v_after_acc,
                                     "x0": next_goal_x, "t0": eta_of_next_goal}
                    print("L90:", count, cruise_params,
                          arrival_time_if_cruise, acc_period_end, acc_period_length)
                    # arrival_time = eta_of_next_goal

                    """
                    ループを抜ける条件
                    ・加速終了時の速度がv_maxを超える（基準1）
                    ・加速区間終了時が次のWPを超えてしまう（基準1）. 
                    ・加速しすぎてノイズより手前のWPまでのいずれかでブレーキが必要になる（基準2）. 
                    """
                    if v_after_acc > car.v_max or acc_period_end > next_goal_x:
                        print("ループ抜ける判定基準1:", v_after_acc,
                              car.v_max, acc_period_end, next_goal_x)
                        start_params = next_start_params
                        break

                    # どこかで引っかかったらという条件.
                    upcoming_wps_before_noise = [{"xe": e["x"], "te": e["eta"]} for i, e in enumerate(
                        earliest_etas) if i >= idx and e["x"] <= noise_avoid_point["xe"]]
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

        else:  # ノイズより後ろのWPsが来た場合.
            # まずはacc_itineraryからノイズの到着時間を計算
            x_at_noise_start = calc_distance_from_acc_itinerary(
                acc_itinerary, noise_avoid_point["te"])
            if x_at_noise_start < noise_avoid_point["xe"]:
                print("最速で行ってもノイズに当たってしまうので早避け不可能")
                print("detail:", x_at_noise_start,
                      noise_avoid_point["xe"], noise_avoid_point["te"])

                return False

            # これは早避けが可能な場合.
            x = fastest_eta["x"]
            # noiseより後ろのwaypointに対しては、その先も衝突しないことを保証する必要がある.
            # 20240724はここから作業開始.
            """
            <前提> これより上のループにより、ノイズの手前まではConstraintsを満たしながら早避けのペースで来ている. 
            <ここからの方針>
            ・start_paramsをnoise_avoid_pointに設定
            ・そこからはnoise以後のearliest_etaを満たすように加速度を調整する. 無理ならFalse
            """
            print("start params: ", start_params)
            eta_boundary = {"xe": fastest_eta["x"], "te": fastest_eta["eta"]}
            car_params = {"decel": car.a_min, "accel": car.a_max}
            should_brake_for_next_interval = will_collide(
                **start_params, **eta_boundary, decel=car.a_min)

            if start_params["x0"] >= eta_boundary["xe"]:
                continue
            # ブレーキを踏む必要がない場合
            if not should_brake(**start_params, **eta_boundary) and not should_brake_for_next_interval:
                continue
            # ブレーキを踏む必要がある上で、踏めば一応大丈夫な場合.
            if can_reach_after_designated_eta(**start_params, **eta_boundary, car_params=car_params):
                print("!!!!!!!!!!")
                print(start_params, eta_boundary, earliest_etas)
                print("!!!!!!!!!!")

                a, eta = crt_acc_itinerary_for_decel_area(
                    # FIXME: 加速度計算の時のstep_sizeは論点
                    **start_params, **eta_boundary, ve=None, car_params=car_params, step_size=0.5, earliest_etas=earliest_etas, car_idx=car.car_idx)
                v = a[-1]["v_0"]
                acc_itinerary = update_acc_itinerary(
                    acc_itinerary, a)
                continue

            else:
                # この場合は、そもそも早避けが不可能なのでFalseを返す.
                # raise ValueError("前の車に当たってしまう. ")
                print("前の車に当たってしまうので早避け不可")
                return False

    return acc_itinerary


"""
先頭車がいない場合はとりあえず最大速度まで加速してそのまま走る. 
"""


def calc_leader_acc_itinerary(car, current_time):
    first_acc_period = (car.v_max - car.v_x) / car.a_max
    acc_itinerary = [{"t_start": current_time, "acc": car.a_max, "x_start": car.xcor,
                      "v_0": car.v_x, "t_end": current_time + first_acc_period}]
    cruise_period = {"t_start": current_time + first_acc_period, "x_start": car.xcor + 0.5 * car.a_max * first_acc_period**2 + car.v_x * first_acc_period,
                     "acc": 0, "v_0": car.v_max, "t_end": 1e7}
    acc_itinerary.append(cruise_period)
    print("leader acc_itinerary:", acc_itinerary)
    return acc_itinerary


def should_brake(v0, x0, t0, xe, te):
    arrival_time = (xe - x0) /( v0+1e-4 ) + t0
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


def calc_eta_from_acc(x_cor, acc_itinerary):
    start_x = 0
    for idx, acc_info in enumerate(acc_itinerary):
        delta_x = x_cor - start_x
        t_start = acc_info["t_start"]
        acc = acc_info["acc"]
        v_0 = acc_info["v_0"]
        # print("calc_eta_from_acc.py; L20",start_x, acc_info, delta_x)

        # 一番最後の場合 (必ず結果が返る)
        if idx == len(acc_itinerary) - 1:
            # print(f"最後: delta_x={delta_x}, acc_info={acc_info}")
            if acc_info["acc"] == 0:
                # 等速の場合
                return delta_x / (v_0 +1e-4)+ t_start
            # 加速度のある場合
            return ((v_0**2 + 2 * acc * delta_x)**0.5 - v_0)/acc + t_start

        # 次の区間が存在する場合
        else:
            # print(f"次あり: delta_x={delta_x}, acc_info={acc_info}")
            duration = acc_itinerary[int(idx+1)]["t_start"] - t_start
            cover_distance = v_0*duration + 0.5 * acc * duration**2

            if delta_x > cover_distance:
                start_x += cover_distance
                continue

            # この区間で終わる場合.
            if acc_info["acc"] == 0:
                return t_start + delta_x / (v_0+1e-4)
            else:
                return ((v_0**2 + 2 * acc * delta_x)**0.5 - v_0)/acc + t_start
