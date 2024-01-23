def create_itinerary_from_acc(**kwagrs):
    car_obj = kwagrs.get("car_obj", None)
    acc_itinerary = car_obj.acc_itinerary
    car_position = car_obj.xcor
    print("==== START CREATING ETA====")
    new_itinerary = []
    for idx, item in enumerate(car_obj.itinerary):
        if item["x"] <= car_position:
            new_itinerary.append(item)
            continue
        else:
            eta = calc_eta_from_acc(item["x"], acc_itinerary)
            new_itinerary.append({**item, "eta":eta})
            
    return new_itinerary

def calc_eta_from_acc(x_cor, acc_itinerary):
    start_x = 0
    print(f"xcor: {x_cor}, acc_itinerary:{acc_itinerary}")
    for idx, acc_info in enumerate(acc_itinerary):
        delta_x = x_cor - start_x
        t_start = acc_info["t_start"]
        acc = acc_info["acc"]
        v_0 = acc_info["v_0"]
        print(delta_x)

        # 一番最後の場合 (必ず結果が返る)
        if idx == len(acc_itinerary) -1:
            if acc_info["acc"] == 0:
                # 等速の場合
                return delta_x / v_0 + t_start
            # 加速度のある場合
            return ((v_0**2 + 2 * acc * delta_x)**0.5 - v_0)/acc + t_start
        
        # 次の区間が存在する場合
        else: 
            duration = acc_itinerary[int(idx+1)]["t_start"] - t_start
            cover_distance = start_x + v_0*duration + 0.5 * acc * duration**2

            if delta_x > cover_distance:
                start_x += cover_distance
                continue
            
            # この区間で終わる場合. 
            if acc_info["acc"] == 0:
                return t_start + delta_x / v_0
            else:
                return ((v_0**2 + 2 * acc * delta_x)**0.5 - v_0)/acc + t_start
          


def test():
    acc_itinerary = [{"t_start": 0, "acc": 3}, {"t_start": 4, "acc": -1}]
    car_position = 25
    current_time = 1
    waypoint_x = 60

if __name__ == "__main__":
    test()