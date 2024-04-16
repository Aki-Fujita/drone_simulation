
def check_multiple_noise_effect(noiseList, eta_table, time):
    return any([check_single_noise_effect(noise, eta_table, time) for noise in noiseList])


"""
speed_logとnoiseの情報から判断する
noiseにあたっていないと判断するには以下の (a) or (b)
(a) noise開始時刻にすでにnoiseの終端座標を過ぎている、
(b) noise終了時刻にまだnoiseに到達していない
速度または加速度のログからノイズとの衝突を判定する
"""


def check_single_noise_effect(noise, carObj, current_time):
    noise_start_time = noise["t"][0]
    noise_end_time = noise["t"][1]

    x_at_noise_start = calc_x_at_pointed_time(
        noise_start_time, carObj, current_time)
    x_at_noise_end = calc_x_at_pointed_time(
        noise_start_time, carObj, current_time)

    print(f"carID: {carObj.car_idx}, Start:{x_at_noise_start}, End:{x_at_noise_end}")
    will_avoid_noise_early = x_at_noise_start > noise["x"][1]
    will_avoid_noise_late = x_at_noise_end < noise["x"][0]
    # print(f"Early:{will_avoid_noise_early}, Late:{will_avoid_noise_late}")
    return not (will_avoid_noise_early or will_avoid_noise_late)


# 加速度のログから X を算出する.
def calc_x_at_pointed_time(pointed_time, carObj, current_time):
    acc_itinerary = carObj.acc_itinerary
    car_x = carObj.xcor
    v_0 = carObj.v_x
    acc_itinerary_with_tend = add_t_end_to_acc_itinerary(acc_itinerary, current_time)
    print("==============")
    print(acc_itinerary)
    print(acc_itinerary_with_tend)
    print("==============")

    if len(acc_itinerary_with_tend) < 1:
        raise ValueError("acc_itinerary is empty")
    if len(acc_itinerary_with_tend) == 1:
        delta_t = pointed_time - current_time
        delta_x = v_0 * delta_t + 0.5 * acc_itinerary_with_tend[0]["acc"] * delta_t**2 + car_x
        return delta_x
    
    # 長さ2以上の場合
    for idx, accObj in enumerate(acc_itinerary_with_tend):
        if accObj["t_end"] < current_time:
            continue

        # この区間を全うできる場合.
        elif accObj["t_end"] < pointed_time:
            delta_t = accObj["t_end"] - accObj["t_start"]
            if accObj["t_start"] < current_time:
                delta_t = accObj["t_end"] - current_time
            delta_x = v_0 * delta_t + 0.5 * accObj["acc"] * delta_t**2
            car_x += delta_x
            v_0 += delta_t * accObj["acc"]

        # 一番最後の区間の場合
        elif idx == len(acc_itinerary) - 1 or accObj["t_end"] > pointed_time:
            delta_t = pointed_time - accObj["t_start"]
            if accObj["t_start"] < current_time:
                delta_t = accObj["t_end"] - current_time
            delta_x = v_0 * delta_t + 0.5 * accObj["acc"] * delta_t**2
            car_x += delta_x
            break
        else:
            print(accObj, current_time, pointed_time)
            raise ValueError("Something wrong")

    return car_x

def add_t_end_to_acc_itinerary(acc_itinerary, current_time):
    result = []
    for idx, accObj in enumerate(acc_itinerary):
        if idx < len(acc_itinerary) - 1:
            accObj["t_end"] = acc_itinerary[idx+1]["t_start"]
            result.append(accObj)
        else:
            accObj["t_end"] = 1e7
            result.append(accObj)
    return result            


class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time", 0)
        self.xcor = 0
        self.v_x = kwagrs.get("v_x")
        self.speed_itinerary = [
            {"speed": self.v_x, "start": self.arrival_time}]  # 速度の更新予定表
        self.acc_itinerary = kwagrs.get("acc_itinerary")


def test():
    print("TEST START")
    current_time = 3
    noise = {"t": [3, 5], "x": [18, 20]}
    acc_itinerary_1 = [{"t_start": 0, "acc": 3}]
    acc_itinerary_2 = [{"t_start": 0, "acc": 0}]
    carObj = Cars(v_x=3, acc_itinerary=acc_itinerary_1)

    result = check_single_noise_effect(noise, carObj)
    print(result)


if __name__ == "__main__":
    test()
