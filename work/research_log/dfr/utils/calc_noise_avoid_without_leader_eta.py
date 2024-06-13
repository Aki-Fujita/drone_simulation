from .optimizer_for_follower import should_brake, update_acc_itinerary_with_accel, update_acc_itinerary, can_reach_after_designated_eta, crt_acc_itinerary_for_decel_area
import copy

def calc_noise_avoid_without_leader_eta(**kwargs):
    car = kwargs.get("car")
    xe = kwargs.get("xe") # noiseの発生位置
    te = kwargs.get("te") # noiseを通過する時刻（ノイズ終了時刻くらい）
    current_time = kwargs.get("current_time") # noiseを通過する時刻（ノイズ終了時刻くらい）
    my_etas = kwargs.get("etas")
    earliest_etas = insert_noise_eta(my_etas, xe, te)
    acc_itinerary = acc_solver(earliest_etas=earliest_etas, car=car, current_time=current_time)
    print(" acc:",acc_itinerary)

    return acc_itinerary

def insert_noise_eta(my_etas, xe, te):
    """
    自分の既存のETAリストにノイズを組み込む（だけ）
    """
    eta_dict_list = my_etas.sort_values(by="x").to_dict(orient="records")
    eta_dict_list.append({"x":xe, "eta":te})
    sorted_data = sorted(eta_dict_list, key=lambda item: (item["x"], -item['eta']))
    # 続いて重複を排除する.
    result = []
    seen = set()
    for item in sorted_data:
        if item['x'] not in seen:
            result.append(item)
            seen.add(item['x'])
    return result

def acc_solver(earliest_etas, car, current_time):
    initial_params = {"v0":car.v_x, "x0":car.xcor, "t0":current_time}
    car_params = {"decel":car.a_min, "accel":car.a_max}
    start_params = copy.deepcopy(initial_params)
    current_itinerary = [{"t_start":current_time, "acc":0, "v_0":car.v_x, "t_end":earliest_etas[0]["eta"]}]
    print("Earliest ETAs: ",earliest_etas)
    for wp_idx, earliest_eta in enumerate(earliest_etas):
        eta_plan = {"xe":earliest_eta["x"], "te":earliest_eta["eta"]}
        print(eta_plan, start_params, should_brake(**start_params, **eta_plan))
        if start_params["x0"] == eta_plan["xe"]:
            continue
        if not should_brake(**start_params, **eta_plan):
            upcoming_wps = [{"xe":e["x"], "te":e["eta"]} for i, e in enumerate(earliest_etas) if i >= wp_idx]
            if all([not should_brake(**start_params, **eta_plan) for eta_plan in upcoming_wps]):
                new_itinerary, sp = update_acc_itinerary_with_accel(current_itinerary, start_params, upcoming_wps, car_params={**car_params, "acc":2})
                current_itinerary = update_acc_itinerary(current_itinerary, new_itinerary)
                start_params = sp
            else:
                continue
            continue
        if can_reach_after_designated_eta(**start_params, **eta_plan, car_params=car_params):
            a, eta = crt_acc_itinerary_for_decel_area(**start_params, **eta_plan, ve=None, car_params=car_params, step_size=0.5) # FIXME: 加速度計算の時のstep_sizeは論点
            v = a[-1]["v_0"]
            start_params = {"v0":v, "x0":eta_plan["xe"], "t0":eta}
            current_itinerary = update_acc_itinerary(current_itinerary, a)
            continue
        
        else:
            raise ValueError("絶対に当たってしまうノイズがあります。")
    
    return current_itinerary
    

def conduct_tests():
    # car=
    # calc_noise_avoid_without_leader_eta(car=car, xe=610, te=50.1, current_time=12, my_etas=etas)
    
    pass

if __name__ == "__main__":
    conduct_tests()