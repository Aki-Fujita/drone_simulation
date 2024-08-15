from .calc_distance_from_acc_itinerary import calc_distance_from_acc_itinerary


def check_multiple_noise_effect(noiseList, eta_table, time):
    return any([not will_avoid_single_noise(noise, eta_table, time) for noise in noiseList])


"""
Input: noise, carObj, current_time
Output: bool, 当たっていたらFalse, そうでなければTrue
speed_logとnoiseの情報から判断する
noiseにあたっていないと判断するには以下の (a) or (b)
(a) noise開始時刻にすでにnoiseの終端座標を過ぎている、
(b) noise終了時刻にまだnoiseに到達していない
速度または加速度のログからノイズとの衝突を判定する
"""


def will_avoid_single_noise(noise, carObj, current_time):
    noise_start_time = noise["t"][0]
    noise_end_time = noise["t"][1]
    if noise_end_time < current_time:
        return True
    if carObj.car_idx == 7:
        print(carObj.acc_itinerary, noise_start_time,
              noise_end_time, current_time, carObj.xcor)

    x_at_noise_start = calc_distance_from_acc_itinerary(
        carObj.acc_itinerary, noise_start_time)
    x_at_noise_end = calc_distance_from_acc_itinerary(
        carObj.acc_itinerary, noise_end_time)

    # print(f"carID: {carObj.car_idx}, Start:{
    #       x_at_noise_start}, End:{x_at_noise_end}")
    will_avoid_noise_early = x_at_noise_start > noise["x"][1]
    will_avoid_noise_late = x_at_noise_end < noise["x"][0]
    # print(f"Early:{will_avoid_noise_early}, Late:{will_avoid_noise_late}")
    return (will_avoid_noise_early or will_avoid_noise_late)


# 加速度のログから X を算出する.
def calc_x_at_pointed_time(pointed_time, carObj, current_time):
    acc_itinerary = carObj.acc_itinerary
    car_x = carObj.xcor
    # print(carObj.xcor, carObj.acc_itinerary)
    v_0 = carObj.v_x
    acc_itinerary_with_tend = add_t_end_to_acc_itinerary(
        acc_itinerary, current_time)
    if carObj.car_idx == 7:
        print(f"carId: {carObj.car_idx}, acc_itinerary: {
              acc_itinerary_with_tend}")

    if len(acc_itinerary_with_tend) < 1:
        raise ValueError("acc_itinerary is empty")
    if len(acc_itinerary_with_tend) == 1:
        delta_t = pointed_time - current_time
        delta_x = v_0 * delta_t + 0.5 * \
            acc_itinerary_with_tend[0]["acc"] * delta_t**2 + car_x
        return delta_x

    # 長さ2以上の場合
    for idx, accObj in enumerate(acc_itinerary_with_tend):
        if accObj["t_end"] < current_time:
            continue

        # この区間を全うできる場合.
        elif accObj["t_end"] <= pointed_time:
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
            print(accObj, current_time, pointed_time, carObj.car_idx)
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
        self.xcor = kwagrs.get("xcor", 0)
        self.v_x = kwagrs.get("v_x")
        self.speed_itinerary = [
            {"speed": self.v_x, "start": self.arrival_time}]  # 速度の更新予定表
        self.acc_itinerary = kwagrs.get("acc_itinerary")


def test():
    print("TEST START")
    acc_itinerary = [{'acc': 0, 't_start': 18.619840967701887, 'v_0': 20, 't_end': 18.8}, {'acc': -0.6151827854343851, 't_start': 18.8, 'v_0': 20, 't_end': 24.0}, {'acc': -0.0004957099016997256, 't_start': 24.0, 'v_0': 16.801049515741198, 't_end': 29.200000000000003}, {'acc': -2.2373518901647887e-06, 't_start': 29.200000000000003, 'v_0': 16.79847182425236, 't_end': 34.400000000000006},
                     {'acc': -0.00031615631758539396, 't_start': 34.400000000000006, 'v_0': 16.79846019002253, 't_end': 39.6}, {'acc': 0.9125191519907546, 't_start': 39.6, 'v_0': 16.796816177171088, 't_end': 44.8}, {'acc': 2.999999999990938, 't_start': 44.8, 'v_0': 21.54191576752301, 't_end': 50}, {'acc': 0, 't_start': 50, 'v_0': 37.14191576747589, 't_end': 10000000.0}]
    car = Cars(v_x=32.341978551, acc_itinerary=acc_itinerary,
               xcor=555.647896385811)
    res = calc_x_at_pointed_time(50, car, 48.4)
    print(res)


if __name__ == "__main__":
    test()
