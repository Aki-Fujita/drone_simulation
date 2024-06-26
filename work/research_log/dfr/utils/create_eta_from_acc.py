def create_itinerary_from_acc(**kwagrs):
    car_obj = kwagrs.get("car_obj", None)
    acc_itinerary = kwagrs.get("acc_itinerary", None)
    car_position = car_obj.xcor
    print("==== START CREATING ETA====")
    print("acc_itinerary:", acc_itinerary)
    new_itinerary = []
    for idx, item in enumerate(car_obj.my_etas):
        if item["x"] <= car_position:
            new_itinerary.append(item)
            continue
        else:
            eta = calc_eta_from_acc(item["x"], acc_itinerary)
            new_itinerary.append({**item, "eta":eta})
    return new_itinerary

def calc_eta_from_acc(x_cor, acc_itinerary):
    start_x = 0
    for idx, acc_info in enumerate(acc_itinerary):
        delta_x = x_cor - start_x
        t_start = acc_info["t_start"]
        acc = acc_info["acc"]
        v_0 = acc_info["v_0"]
        # print("calc_eta_from_acc.py; L20",start_x, acc_info, delta_x)

        # 一番最後の場合 (必ず結果が返る)
        if idx == len(acc_itinerary) -1:
            # print(f"最後: delta_x={delta_x}, acc_info={acc_info}")
            if acc_info["acc"] == 0:
                # 等速の場合
                return delta_x / v_0 + t_start
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
                return t_start + delta_x / v_0
            else:
                return ((v_0**2 + 2 * acc * delta_x)**0.5 - v_0)/acc + t_start
          


def test():
    acc_itinerary = [{'t_start': 4.0, 'acc': 3, 'v_0': 20}, {'t_start': 4.2040863037109375, 'acc': 0, 'v_0': 20.612258911132812}, {'t_start': 39.21782118993376, 'acc': -3, 'v_0': 20.612258911132812}, {'t_start': 39.421907493644696, 'acc': 0, 'v_0': 20}]
    car_position = 25
    current_time = 1
    waypoint_x = 60
    eta = calc_eta_from_acc(730,acc_itinerary)
    print(eta)

if __name__ == "__main__":
    test()