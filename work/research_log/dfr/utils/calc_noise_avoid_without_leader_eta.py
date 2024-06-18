from .optimizer_for_follower import should_brake, update_acc_itinerary_with_accel, update_acc_itinerary, can_reach_after_designated_eta, crt_acc_itinerary_for_decel_area, merge_acc_itinerary, will_collide
import copy

# これは late_avoidの場合の関数


def calc_noise_avoid_without_leader_eta(**kwargs):
    car = kwargs.get("car")
    xe = kwargs.get("xe")  # noiseの発生位置
    te = kwargs.get("te")  # noiseを通過する時刻（ノイズ終了時刻くらい）
    current_time = kwargs.get("current_time")  # noiseを通過する時刻（ノイズ終了時刻くらい）
    my_etas = kwargs.get("etas")
    earliest_etas = insert_noise_eta(my_etas, xe, te)
    acc_itinerary = acc_solver(
        earliest_etas=earliest_etas, car=car, current_time=current_time)
    print(" acc:", acc_itinerary)
    merged_acc_itinerary = merge_acc_itinerary(
        pre_itinerary=car.acc_itinerary, new_itinerary=acc_itinerary)
    print("merged:", merged_acc_itinerary)
    return merged_acc_itinerary


def insert_noise_eta(my_etas, xe, te):
    """
    自分の既存のETAリストにノイズを組み込む（だけ）
    """
    eta_dict_list = my_etas.sort_values(by="x").to_dict(orient="records")
    eta_dict_list.append({"x": xe, "eta": te})
    sorted_data = sorted(
        eta_dict_list, key=lambda item: (item["x"], -item['eta']))
    # 続いて重複を排除する.
    result = []
    seen = set()
    for item in sorted_data:
        if item['x'] not in seen:
            result.append(item)
            seen.add(item['x'])
    return result


def acc_solver(earliest_etas, car, current_time):
    initial_params = {"v0": car.v_x, "x0": car.xcor, "t0": current_time}
    car_params = {"decel": car.a_min, "accel": car.a_max}
    start_params = copy.deepcopy(initial_params)
    itinerary_from_now = [{"t_start": current_time, "acc": 0,
                           "v_0": car.v_x, "t_end": earliest_etas[0]["eta"]}]
    print("Earliest ETAs: ", earliest_etas, itinerary_from_now)
    for wp_idx, earliest_eta in enumerate(earliest_etas):
        eta_boundary = {"xe": earliest_eta["x"], "te": earliest_eta["eta"]}
        should_brake_for_next_interval = will_collide(
            **start_params, **eta_boundary, decel=car_params["decel"])
        print("次のWPまでの情報: 境界条件", eta_boundary, "初期条件:", start_params,
              "should_brake", should_brake(
                  **start_params, **eta_boundary), "will_collide:", should_brake_for_next_interval
              )
        if start_params["x0"] == eta_boundary["xe"]:
            continue
        # ブレーキを踏む必要がない場合
        if not should_brake(**start_params, **eta_boundary) and not should_brake_for_next_interval:
            upcoming_wps = [{"xe": e["x"], "te": e["eta"]}
                            for i, e in enumerate(earliest_etas) if i >= wp_idx]
            if all([not should_brake(**start_params, **eta_boundary) for eta_boundary in upcoming_wps]):
                new_itinerary, sp = update_acc_itinerary_with_accel(itinerary_from_now, start_params, upcoming_wps, car_params={
                                                                    **car_params, "acc": 2, "v_max": car.v_max}, current_x=car.xcor)
                print("result of 'update_acc_itinerary_with_accel':  ",
                      new_itinerary, "\n sp:", sp)
                itinerary_from_now = update_acc_itinerary(
                    itinerary_from_now, new_itinerary)
                start_params = sp
            else:
                continue
            continue
        if can_reach_after_designated_eta(**start_params, **eta_boundary, car_params=car_params):
            a, eta = crt_acc_itinerary_for_decel_area(
                # FIXME: 加速度計算の時のstep_sizeは論点
                **start_params, **eta_boundary, ve=None, car_params=car_params, step_size=0.5, earliest_etas=earliest_etas)
            v = a[-1]["v_0"]
            start_params = {"v0": v, "x0": eta_boundary["xe"], "t0": eta}
            itinerary_from_now = update_acc_itinerary(itinerary_from_now, a)
            continue

        else:
            raise ValueError("絶対に当たってしまうノイズがあります。")

    return itinerary_from_now


def conduct_tests():
    # car=
    # calc_noise_avoid_without_leader_eta(car=car, xe=610, te=50.1, current_time=12, my_etas=etas)

    pass


if __name__ == "__main__":
    conduct_tests()
