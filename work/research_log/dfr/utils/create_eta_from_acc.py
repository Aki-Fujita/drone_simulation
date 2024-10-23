def create_itinerary_from_acc(**kwagrs):
    car_obj = kwagrs.get("car_obj", None)
    acc_itinerary = kwagrs.get("acc_itinerary", None)
    car_position = car_obj.xcor
    # print("L6: acc_itinerary:", acc_itinerary)
    new_itinerary = []
    for idx, item in enumerate(car_obj.my_etas):
        if item["x"] <= car_position:
            new_itinerary.append(item)
            continue
        else:
            eta = calc_eta_from_acc(item["x"], acc_itinerary)
            new_itinerary.append({**item, "eta": eta})
    return new_itinerary


def calc_eta_from_acc(x_coor, acc_itinerary):
    """
    Input: (目標地点のX座標, 加速度の情報)
    Output: 目標地点に到達するETA
    【処理概要】
    acc_itineraryを元にXが更新されるたびに最初（acc_itineraryの初項）から計算する.

    # 速く計算できるケース
    x_startが入力されている場合は最も近いx_startから計算する. 
    """

    acc_itinerary = sorted(acc_itinerary, key=lambda x: x["t_start"])

    # x_coor に一致するものをチェック (許容誤差 1e-2)
    for acc_info in acc_itinerary:
        if abs(acc_info.get("x_start", float('-inf')) - x_coor) < 1e-2:
            return acc_info["t_start"]

    closest_x_start = 0
    closest_index = 0
    for idx, acc_info in enumerate(acc_itinerary):
        x_start = acc_info.get("x_start", float('inf'))
        if x_start < x_coor and closest_x_start < x_start:
            closest_x_start = x_start
            closest_index = idx

    # closest_x_start からx_coorまでのETAを計算する
    total_time = 0
    for idx, acc_info in enumerate(acc_itinerary):
        if idx < closest_index:
            continue

        delta_x = x_coor - closest_x_start
        t_start = acc_info["t_start"]
        v_0 = acc_info["v_0"]
        acc = acc_info["acc"]

        # 現在の区間が最後かどうかをチェック
        if idx == len(acc_itinerary) - 1:
            # 最後の区間の処理
            if acc == 0:
                # 等速の場合
                return delta_x / (v_0+1e-4) + t_start
            else:
                # 加速度がある場合
                return ((v_0**2 + 2 * acc * delta_x) ** 0.5 - v_0) / acc + t_start
        else:
            # 次の区間が存在する場合
            next_t_start = acc_itinerary[idx + 1]["t_start"]
            duration = next_t_start - t_start
            covered_distance = v_0 * duration + 0.5 * acc * duration ** 2

            if delta_x <= covered_distance:
                # この区間で目標に到達できる場合
                if acc == 0:
                    return t_start + delta_x / (v_0 + 1e-4)
                else:
                    return ((v_0**2 + 2 * acc * delta_x) ** 0.5 - v_0) / acc + t_start

            # この区間を全て通過する場合
            closest_x_start += covered_distance

    # 目標地点に到達するETAを計算
    return total_time


def test():
    acc_itinerary = [{'t_start': 4.0, 'acc': 3, 'v_0': 20}, {'t_start': 4.2040863037109375, 'acc': 0, 'v_0': 20.612258911132812}, {
        't_start': 39.21782118993376, 'acc': -3, 'v_0': 20.612258911132812}, {'t_start': 39.421907493644696, 'acc': 0, 'v_0': 20}]
    car_position = 25
    current_time = 1
    waypoint_x = 60
    eta = calc_eta_from_acc(730, acc_itinerary)
    print(eta)


if __name__ == "__main__":
    test()
