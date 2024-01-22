
def check_multiple_noise_effect(noiseList, eta_table):
    return any([check_single_noise_effect(noise, eta_table) for noise in noiseList])


"""
speed_logとnoiseの情報から判断する
noiseにあたっていないと判断するには以下の (a) or (b)
(a) noise開始時刻にすでにnoiseの終端座標を過ぎている、
(b) noise終了時刻にまだnoiseに到達していない
速度または加速度のログからノイズとの衝突を判定する
"""


def check_single_noise_effect(noise, carObj):
    noise_start_time = noise["t"][0]
    noise_end_time = noise["t"][1]

    x_at_noise_start = calc_x_at_pointed_time(
        noise_start_time, carObj.acc_itinerary, carObj.xcor, carObj.v_x)
    x_at_noise_end = calc_x_at_pointed_time(
        noise_end_time, carObj.acc_itinerary, carObj.xcor, carObj.v_x)

    print(f"Start:{x_at_noise_start}, End:{x_at_noise_end}")
    will_avoid_noise_early = x_at_noise_start > noise["x"][1]
    will_avoid_noise_late = x_at_noise_end < noise["x"][0]
    print(f"Early:{will_avoid_noise_early}, Late:{will_avoid_noise_late}")
    return will_avoid_noise_early or will_avoid_noise_late


# 加速度のログから X を算出する.
def calc_x_at_pointed_time(pointed_time, acc_itinerary, initial_x, initial_v):
    acc_itinerary = acc_itinerary
    car_x = initial_x
    v_0 = initial_v

    for idx, accObj in enumerate(acc_itinerary):
        # 一番最後の要素だった場合はendTime = noise_start_time
        print(accObj)
        print(f"v = {v_0}, x={car_x}")

        if idx == len(acc_itinerary) - 1:
            delta_t = pointed_time - accObj["start"]
            delta_x = v_0 * delta_t + 0.5 * accObj["acc"] * delta_t**2
            car_x += delta_x
            print(f"A. 持続時間:{delta_t}, この区間で進む距離{delta_x}")

            break
        # 次のitineraryがあるけどそれより前にnoiseが来る時.
        elif acc_itinerary[idx+1]["start"] > pointed_time:
            print("B")
            delta_t = pointed_time - accObj["start"]
            delta_x = v_0 * delta_t + 0.5 * accObj["acc"] * delta_t**2
            car_x += delta_x
            print(f"B. 持続時間:{delta_t}, この区間で進む距離{delta_x}")
            break

        # この区間を全うできる場合.
        else:
            delta_t = acc_itinerary[idx+1]["start"] - accObj["start"]
            delta_x = v_0 * delta_t + 0.5 * accObj["acc"] * delta_t**2
            car_x += delta_x
            v_0 += delta_t * accObj["acc"]
            print(f"C. 持続時間:{delta_t}, この区間で進む距離{delta_x}")

    return car_x


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
    acc_itinerary_1 = [{"start": 0, "acc": 3}]
    acc_itinerary_2 = [{"start": 0, "acc": 0}]
    carObj = Cars(v_x=3, acc_itinerary=acc_itinerary_1)

    result = check_single_noise_effect(noise, carObj)
    print(result)


if __name__ == "__main__":
    test()
