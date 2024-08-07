from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
import math
# from calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary #単体テスト実行時はこっち
# 親ファイルから呼ぶときはこっち
from .calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary
from .simple_funcs import will_collide
import copy

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
    valid_items = sorted([item for item in data if item['t_start']
                         < t], key=lambda x: x['t_start'], reverse=True)
    # 条件を満たす最初の要素の acc を返す
    return valid_items[0]['acc'] if valid_items else None


"""
前の車もnoiseを late_avoidしている前提で作られている！
"""


def calc_late_avoid_with_leader(**kwargs):
    follower = kwargs.get("follower")
    ttc = kwargs.get("ttc", 0)
    leader = kwargs.get("leader", 0)  # 先行車のオブジェクト
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

    # 後続車のパラメータ
    acc_itinerary = follower_acc_solver(
        follower, eta_of_leader, ttc, current_time)
    merged_acc_itinerary = merge_acc_itinerary(
        pre_itinerary=follower.acc_itinerary, new_itinerary=acc_itinerary)
    print("merged:", merged_acc_itinerary)
    return merged_acc_itinerary, time_step, steps


def follower_acc_solver(follower, eta_of_leader, TTC, current_time):
    """
    基本方針
    (a) まずleaderのETAを一通り確認し、それに対し+TTC秒以上開けたETAを作成する
    (b) 続いてそのETAがvalidであるかを検証し、その上で加速度のitineraryを作成する
    """
    # print("LEADER ETA: \n", eta_of_leader.to_dict(orient="records"))
    eta_of_leader = eta_of_leader.sort_values(
        by=["x"]).to_dict(orient="records")

    # (a) まずはLeaderのETAから自分に可能な最速のETAを計算する（前準備）
    earliest_etas = []
    for eta_info in eta_of_leader:
        earilist_eta = eta_info["eta"] + TTC + \
            0.15  # 先頭車のETAにTTC秒以上開けた上でマージンを0.15秒とる
        earliest_etas.append(
            {**eta_info, "eta": earilist_eta, "car_idx": follower.car_idx})

    # (b) 続いて上のETAが実現可能であるかを検証し、無理ならばETAを遅らせていく
    initial_params = {"v0": follower.v_x,
                      "x0": follower.xcor, "t0": current_time}
    car_params = {"decel": follower.a_min,
                  "accel": follower.a_max, "v_max": follower.v_max}
    start_params = copy.deepcopy(initial_params)
    itinerary_from_now = [{"t_start": current_time, "acc": 0,
                           "v_0": follower.v_x, "t_end": earliest_etas[0]["eta"]}]
    print("late avoid with leader: boundary:",
          earliest_etas, "\n itinerary:", itinerary_from_now)

    for wp_idx, earliest_eta in enumerate(earliest_etas):

        eta_boundary = {"xe": earliest_eta["x"], "te": earliest_eta["eta"]}
        if earliest_eta["x"] < follower.xcor:
            # print("この区間はもう通り過ぎている")
            continue

        if not should_brake(**start_params, **eta_boundary):
            print("L116. sp: ", start_params, eta_boundary)
            # この場合、この区間のさらに先まで見た上で、これから先どこもブレーキが必要なかったら加速する！
            upcoming_wps = [{"xe": e["x"], "te": e["eta"]}
                            for i, e in enumerate(earliest_etas) if i >= wp_idx]
            if all([not should_brake(**start_params, **eta_plan) for eta_plan in upcoming_wps]):
                # print("これだとだいぶ余裕があるので加速する")
                new_itinerary, sp = update_acc_itinerary_with_accel(itinerary_from_now, start_params, upcoming_wps, car_params={
                                                                    **car_params, "acc": 2}, current_x=follower.xcor)

                # 今のacc_itineraryだとこの先にブレーキが必要ないので、どこかしらに当たるまで加速する.
                itinerary_from_now = update_acc_itinerary(
                    itinerary_from_now, new_itinerary)
                start_params = sp
            else:
                # ブレーキをかけずに走ることが確定しているが、次の区間でブレーキを踏む可能性がまだある.
                continue
            continue
        if can_reach_after_designated_eta(**start_params, **eta_boundary, car_params=car_params):
            print(f'減速すべき区間, goal:{eta_boundary["xe"]}, sp:{start_params}')
            a, eta = crt_acc_itinerary_for_decel_area(
                **start_params, **eta_boundary, ve=None, car_params=car_params, step_size=0.5, earliest_etas=earliest_etas)
            v = a[-1]["v_0"]  # 必ず等速区間で終わるため
            start_params = {"v0": v, "x0": eta_boundary["xe"], "t0": eta}
            itinerary_from_now = update_acc_itinerary(itinerary_from_now, a)
            continue

        else:
            raise ValueError("どうやってもETAを満たせない")

    return itinerary_from_now


def merge_acc_itinerary(pre_itinerary, new_itinerary):
    """
    2つのacc_itineraryをマージする
    """
    result = copy.deepcopy(pre_itinerary)
    print("pre_itinerary: ", pre_itinerary, "new_itinerary: ", new_itinerary)
    # そもそもnew_itineraryの方が前から始まっていたらnewを正とする.
    if new_itinerary[0]["t_start"] <= pre_itinerary[0]["t_start"]:
        return new_itinerary

    # 完全に後のものが入ってきたらそのまま追加
    if pre_itinerary[-1]["t_end"] <= new_itinerary[0]["t_start"]:
        result[-1]["t_end"] = new_itinerary[0]["t_start"]
        result = pre_itinerary + new_itinerary
        return result

    # その他: 途中をreplaceすることになる場合.
    retList = []
    if new_itinerary[0]["t_start"] < pre_itinerary[-1]["t_end"]:
        # これは要するにどこかの区間の途中で入ってきているということなので、new_itineraryが入ってからはそっちを正とする.
        new_itinerary_start = new_itinerary[0]["t_start"]
        for pre_item in pre_itinerary:
            if pre_item["t_end"] < new_itinerary_start:
                retList.append(pre_item)
                continue
            else:  # ここからはnew_itineraryの情報を使う
                fixed_pre_item = {**pre_item, "t_end": new_itinerary_start}
                retList.append(fixed_pre_item)
                break
        retList += new_itinerary
        # print("retList: ", retList)
        return retList

    return result


def update_acc_itinerary(current_itinerary, new_itinerary):
    """
    itineraryのappend処理をする
    加速度が同じだったら区間を繋げる
    """
    result = copy.deepcopy(current_itinerary)
    # そもそもnew_itineraryの方が前から始まっていたらnew_itineraryで完全にreplaceする.
    if new_itinerary[0]["t_start"] == current_itinerary[0]["t_start"]:
        return new_itinerary
    # 末尾と先頭の加速度が等しい場合は区間を繋げる
    if result[-1]["acc"] == new_itinerary[0]["acc"]:
        result[-1]["t_end"] = new_itinerary[0]["t_end"]
    else:
        result = current_itinerary + new_itinerary
    return result


def should_brake(v0, x0, t0, xe, te):
    """
    よく使う基本関数
    初期条件{v0, x0, t0}と目標条件{ve, xe, te}から, 自分がブレーキすべきかどうかを判断する
    initial_paramsのままゴールまで進んだ時に到着時間がt_eを超えるかどうかを計算する
    """
    arrival_time = (xe - x0) / v0 + t0
    # print(f"Arrival Time: {arrival_time}, te: {te}, v0:{v0}, x0:{x0}, t0:{t0}")
    if arrival_time > te:  # 普通に等速で進んでもETAより後ろになる場合はブレーキをかけないで良い.
        return False
    return True


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
    decel = -1 * abs(car_params.get("decel", None))  # 減速度には負の値を入れる.
    brake_time = v0 / abs(decel)
    brake_distance = (v0**2)/(2*abs(decel))  # 速度が0になるまでの時間
    if delta_t <= brake_time:  # 速度が0にならない場合
        cover_distance = v0 * delta_t + 0.5 * decel * delta_t**2
        if cover_distance < delta_x:
            return True
        else:
            return False  # 全力でブレーキをかけてもdelta_t秒後にはゴールについてしまう.
    else:  # 速度を0にできる場合
        if brake_distance < delta_x:
            return True
        else:
            return False


"""
2つのwaypoint間の進み方を決めるための関数.
can_reach_after_designated_etaがTrueだった場合に、その条件下での最適な加速度を計算する.
【重要】解の形は 減速 => 等速 に必ずなる. 
Output: acc_itinerary # この区間の走り方であることに注意（最初からではない！）
"""


def crt_acc_itinerary_for_decel_area(v0, x0, t0, ve, xe, te, car_params, step_size, earliest_etas):
    decel = abs(car_params.get("decel", None))*-1
    steps = int((te - t0) / step_size) + 1  # 始点から終点までの秒数
    cover_distance = 1e8  # x0から定義したacc_itineraryで進む距離
    loop_count = 0
    delta_x = xe - x0
    while cover_distance > xe - x0 or loop_count < steps:
        print("Loops: ", loop_count)
        loop_count += 1
        # まずは加速度を変更してみる
        decel_period = loop_count * step_size
        acc_itinerary = [{"t_start": t0, "acc": decel,
                          "v_0": v0, "t_end": t0+decel_period}]  # 減速区間分
        acc_itinerary.append({"t_start": t0 + decel_period, "acc": 0,
                             "v_0": v0 + decel * decel_period, "t_end": te})  # 減速後の等速区間分
        # acc_itineraryからt=teでの位置を計算する
        cover_distance = calc_distance_from_acc_itinerary(
            acc_itinerary, te)  # ここで入れているacc_itineraryはあくまでもこの区間の走り方であることに注意
        edge_params = {"v0": v0 + decel * decel_period, "x0": xe, "t0": te}
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
        if cover_distance < xe - x0 and not should_decel_more:
            # acc_itineraryを元に次のwaypointのETAを計算する.
            last_v = acc_itinerary[-1]["v_0"]
            eta = (delta_x - cover_distance) / last_v + te
            acc_info_to_append = copy.deepcopy(acc_itinerary[-1])
            acc_info_to_append["t_end"] = eta
            acc_itinerary[-1] = acc_info_to_append
            return acc_itinerary, eta

    return {}, False


def update_acc_itinerary_with_accel(itinerary_from_now, start_params, upcoming_wps, car_params, current_x):
    """
    遅すぎて余裕のあるacc_itineraryを受け取り、その後に加速区間を追加する.
    Input: acc_itinerary, start_params, upcoming_wps
           upcomint_wps: 加速開始の区間（今いる区間）をi=0とし、それ以降のwaypointsを含む
    Output: updated_acc_itinerary

    具体的なユースケース
    (例) 
    例えば区間2=>3でブレーキが必要だったので2=>3ではブレーキをかけたが、
    その状態だと区間3=>4とそれ以降で余裕がありすぎるので、3=>4は少し加速したい. 
    その際、どれだけ加速するのが適切かを計算するための関数. （加速しすぎは結局後でブレーキを踏むため）

    方針
    while should_brakeがFalseの間
      (0) 加速区間の長さを設定（加速区間の長さは今いる区間の長さにする）
      (1) acc_itineraryの最後に定められた長さの加速区間を追加
      (2) その後に等速区間を追加
      【ここからチェック工程】
        「upcoming_waypointsに対して、全て時間内に行ける」ならまだ加速できるのでwhile続行
        無理ならば、その時点でのacc_itineraryを返す.
    """
    count = 0
    step_size = 0.25
    xe = upcoming_wps[0]["xe"]  # この区間のゴール
    te = upcoming_wps[0]["te"]  # この時刻より早くxeに到達してはいけない.
    x_current = start_params["x0"]  # この区間のスタート
    v_current = start_params["v0"]
    t_current = start_params["t0"]
    result_sp = {**start_params, "x0": xe,
                 "t0": (xe - x_current) / v_current + t_current}
    result = copy.deepcopy(itinerary_from_now)
    result[-1]["t_end"] = result_sp["t0"]
    print("first: start_params.", result_sp)

    while count < 100:
        count += 1
        updated_acc_itinerary = copy.deepcopy(itinerary_from_now)
        acc_period = count * step_size
        acc_segment = {"t_start": itinerary_from_now[-1]["t_end"], "acc": car_params["acc"],
                       "v_0": itinerary_from_now[-1]["v_0"], "t_end": itinerary_from_now[-1]["t_end"] + acc_period}
        updated_acc_itinerary.append(acc_segment)
        acc_segment_end = calc_distance_from_acc_itinerary(
            updated_acc_itinerary, acc_segment["t_end"]) + current_x
        # print("in while loop", count, result_sp, acc_segment_end)
        speed_after_accel = acc_segment["v_0"] + car_params["acc"] * acc_period
        if acc_segment_end > xe or speed_after_accel > car_params["v_max"]:
            return result, result_sp
        cruise_segment = {"t_start": acc_segment["t_end"], "acc": 0, "v_0": speed_after_accel, "t_end": (
            xe - acc_segment_end) / acc_segment["v_0"] + acc_segment["t_end"]}

        updated_acc_itinerary.append(cruise_segment)
        position_at_te = calc_distance_from_acc_itinerary(
            updated_acc_itinerary, te)
        # print(f"count={count}, Position at te: {position_at_te}, xe: {xe}, acc_segment_end: {acc_segment_end}")
        if position_at_te <= xe and acc_segment_end <= xe:
            # この区間は大丈夫なので、これ以降のwaypointsに対しても大丈夫かを判定する.
            # print()
            # print("計算に使った要素:", updated_acc_itinerary,
            #       acc_segment_end, xe, cruise_segment)
            start_params_at_edge = {"v0": cruise_segment["v_0"], "x0": xe, "t0": (
                xe - acc_segment_end)/cruise_segment["v_0"] + cruise_segment["t_start"]}
            # print("should brake: ", start_params_at_edge,  [should_brake(
            #     **start_params_at_edge, **eta_plan) for eta_plan in upcoming_wps], upcoming_wps)
            should_continue = all([not should_brake(
                **start_params_at_edge, **eta_plan) for eta_plan in upcoming_wps])
            # print(should_continue)
            if not should_continue:
                return result, result_sp
            result = updated_acc_itinerary
            # 続いてこれ以降のwaypointに対して、相変わらずshould_brakeがFalseであるかを確認する.
            # ここでまだ全てFalseだったら while ループを続行する.
            result_sp = start_params_at_edge
        else:  # これ以上加速しちゃダメということ.
            return result, result_sp


def isRssOK(distance, leader_speed, follower_speed):
    leader_blake_max = 3
    follower_blake_max = 3
    leader_blake_distance = leader_speed ** 2 / \
        (2 * leader_blake_max)  # ここは一旦ブレーキの最大加速度を3としている
    follower_blake_distance = follower_speed ** 2 / (2 * follower_blake_max)
    if distance + leader_blake_distance < follower_blake_distance:
        print("===FAIL DETAIL===")
        print(distance, leader_blake_distance, follower_blake_distance,
              "leader speed:", leader_speed, "follower speed:", follower_speed)
        print("=================")
    return distance + leader_blake_distance > follower_blake_distance


def conduct_tests():
    te = 20
    print("===TEST START=====")
    a, eta = crt_acc_itinerary_for_decel_area(
        **{"v0": 20, "x0": 0, "t0": 8, "ve": 40, "xe": 110, "te": te, "car_params": {"decel": 2}, "step_size": 0.5})
    print("===TEST END=====")
    print(calc_distance_from_acc_itinerary(a, te))
    print(f"result:{a, eta}")


if __name__ == "__main__":
    conduct_tests()
