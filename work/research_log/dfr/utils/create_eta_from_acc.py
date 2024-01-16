def create_eta_from_acc(**kwagrs):
    car_obj = kwagrs.get("car_obj", None)
    current_time =  kwagrs.get("current_time", None)
    acc_itinerary = car_obj.acc_itinerary
    car_position = car_obj.xcor
    print("==== START CREATING ETA====")
    print(acc_itinerary)
    """
    (a) まずはnoise出口までを作る. 
    """
    new_itinerary = []
    for idx, item in enumerate(car_obj.itinerary):
        if item["x"] <= car_position:
            new_itinerary.append(item)
            continue
        else:
            solve_arrival_time(car_obj, current_time, item["x"])
            
            
    return new_itinerary

def solve_arrival_time(car_obj, current_time, waypoint_x):
    x_t_func = []
    v = car_obj.v_x
    acc_itinerary = car_obj.acc_itinerary
    """
    1. 前処理: 加速度Objに対して、その区間で行ける距離、終了時刻を入れる
    2. それを元に求解
    """
    new_acc_itinerary = []
    previous_speed = v
    for idx, acc_info in enumerate(acc_itinerary):
        if idx == len(acc_itinerary):
            #最後だった場合は "end"列を無限にする
            new_acc_itinerary.append({**acc_info, "end":1e8, "x_end":1e8, "v_0": previous_speed})
            continue
        if idx == 0:
            # 最初の場合
            continue    

    return 


def test():
    acc_itinerary = [{"start": 0, "acc": 3}, {"start": 4, "acc": -1}]
    car_position = 25
    current_time = 1
    waypoint_x = 60

    solve_arrival_time()

if __name__ == "__main__":
    test()