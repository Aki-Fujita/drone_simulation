import copy
import numpy as np
from .calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary
from .will_collide import will_collide
import logging
logging.basicConfig(level=logging.INFO)  # INFOレベル以上を表示


def crt_acc_itinerary_for_decel_area(v0, x0, t0, ve, xe, te, car_params, step_size, earliest_etas, car_idx):
    """
    2つのwaypoint間の進み方を決めるための関数.
    can_reach_after_designated_etaがTrueだった場合に、その条件下での最適な加速度を計算する.
    【重要】解の形は 減速 => 等速 に必ずなる.
    Output: acc_itinerary # この区間の走り方であることに注意（最初からではない！）
    """
    decel = abs(car_params.get("decel", None))*-1
    steps = int((te - t0) / step_size) + 1  # 始点から終点までの秒数
    cover_distance = 1e8  # x0から定義したacc_itineraryで進む距離
    loop_count = 0
    delta_x = xe - x0

    while cover_distance > xe - x0 or loop_count < steps:
        # print("Loops: ", loop_count, steps)
        loop_count += 1
        # 減速区間を増やす
        decel_period = loop_count * step_size

        # 減速区間をすぎた結果速度が0になる場合 または etaがteよりもはるかに遅い場合.
        if v0 + decel * decel_period < 0:
            """
            この場合はtarget_timeまでゴール手前で止まるacc_itineraryを計算する.
            """
            # print("L271: 止まる方に入った")
            if can_stop_before_goal(v0, x0, t0, xe, te, car_params):
                acc_itinerary = stop_at_goal(v0, x0, t0, xe, te, car_params)
                # print(f"L275: acc_itinerary with stop, acc_itinerary={
                #       acc_itinerary}, te={te}")
                
                return acc_itinerary, te

        acc_itinerary = [{"t_start": t0, "acc": decel, "x_start": x0,
                          "v_0": v0, "t_end": t0+decel_period}]  # 減速区間分
        acc_itinerary.append({"t_start": t0 + decel_period, "acc": 0, "x_start": x0 + v0 * decel_period + 0.5 * decel * decel_period**2,
                             "v_0": v0 + decel * decel_period, "t_end": te})  # 減速後の等速区間分
        # acc_itineraryからt=teでの位置を計算する
        cover_distance = calc_distance_from_acc_itinerary(
            acc_itinerary, te)  # ここで入れているacc_itineraryはあくまでもこの区間の走り方であることに注意

        temp_eta = te
        if cover_distance < delta_x:
            last_v = acc_itinerary[-1]["v_0"]
            temp_eta = (delta_x - cover_distance) / (last_v+1e-4) + te
        edge_params = {"v0": v0 + decel *
                       decel_period, "x0": xe, "t0": temp_eta}
        # xeより後ろのwaypointがない場合.
        if [item for item in earliest_etas if item["x"] > xe] == []:
            arrival_time = ((xe - x0) - cover_distance) / \
                edge_params["v0"] + te
            if arrival_time > te:
                return acc_itinerary, te
            else:
                continue

        earliest_eta_of_next_wp = sorted(
            [item for item in earliest_etas if item["x"] > xe], key=lambda x: x['x'], reverse=False)[0]
        # print("テスト", earliest_eta_of_next_wp)  # xeのさらにもう一つ次のWPの到着時間
        next_boundary = {"xe": earliest_eta_of_next_wp["x"],
                         "te": earliest_eta_of_next_wp["eta"]}
        # cover_distance <= xe - x0 かつ、「ブレーキをしなかったことによって次の区間で追突」しなければOK という条件にする
        should_decel_more = will_collide(
            **edge_params, decel=car_params["decel"], **next_boundary,)
        # if car_idx == 6:
        #     print("=====================DEBUG=====================")
        #     print(f"L66: should_decel_more={should_decel_more}, edge_params={
        #           edge_params}, next_boundary={next_boundary}")
        if cover_distance < xe - x0 and not should_decel_more:
            # acc_itineraryを元に次のwaypointのETAを計算する.
            last_v = acc_itinerary[-1]["v_0"]
            eta = (delta_x - cover_distance) / (last_v+1e-4) + te
            # teより5秒以上遅くなる場合は止まる方に変更.
            if eta - te > 5:
                logging.debug(f"ETA >> TE :ETA-TE={eta-te}")
                logging.debug("L307: ETAがteより遅すぎるので止まる方に変更")
                if can_stop_before_goal(v0, x0, t0, xe, te, car_params):
                    acc_itinerary = stop_at_goal(
                        v0, x0, t0, xe, te, car_params)
                    logging.debug(f"acc_itinerary with stop, acc_itinerary={
                          acc_itinerary}")
                    return acc_itinerary, te
            acc_info_to_append = copy.deepcopy(acc_itinerary[-1])
            acc_info_to_append["t_end"] = eta
            acc_itinerary[-1] = acc_info_to_append
            return acc_itinerary, eta

    return {}, False


def can_stop_before_goal(v0, x0, t0, xe, te, car_params):
    """
    (x0, t0) => (x3, te)にそもそも止まれるかを計算.
    ただし、t=teちょうどに止まる保証はない.
    """
    decel = abs(car_params.get("decel", None))
    delta_x = xe - x0
    if (v0 ** 2 / 2 / decel <= delta_x) and v0 < decel * (te - t0):
        return True
    return False


def stop_at_goal(v0, x0, t0, xe, te, car_params):
    """
    can_stop_before_goalがTrueの場合に、(x0, t0) => (x3, te)に至るまでのacc_itinerayを計算する
    # goal = xe-0.1としているがこの実装だと複数台が信号前で並ぶ時に重なってしまう.
    """
    decel = abs(car_params.get("decel", None))
    goal = xe - 0.1  # 0.1m手前で止まる
    stop_distance = v0**2 / 2 / decel  # ブレーキをかけながら進む距離
    braking_start_position = goal - stop_distance
    coasting_distance = braking_start_position - x0
    coasting_period = coasting_distance / (v0 +1e-4)
    decel_period = v0 / decel
    acc_itinerary = [{"t_start": t0, "acc": 0, "x_start": x0,
                      "v_0": v0, "t_end": t0+coasting_period}]  # ブレーキをかけ始めるまで
    acc_itinerary.append({"t_start": t0+coasting_period, "acc": -1*decel, "x_start": x0 + coasting_period*v0,
                         "v_0": v0, "t_end": t0 + coasting_period + decel_period})
    acc_itinerary.append({"t_start": t0 + coasting_period + decel_period, "acc": 0, "x_start": goal,
                         "v_0": 0, "t_end": te})  # 停止中
    acc_itinerary.append(
        {"t_start": te, "acc": 0, "v_0": 1e-1, "t_end": te+1e-1, "x_start": goal})
    return acc_itinerary
