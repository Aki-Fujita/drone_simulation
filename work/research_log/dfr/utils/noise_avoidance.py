# from ..models import Cars
# import sys
# sys.path.append("../")

class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time", 0)
        self.xcor = 0
        self.v_x = kwagrs.get("v_x")
        self.a_max = kwagrs.get("a_max")
        self.max_speed = kwagrs.get("max_speed")
        self.speed_itinerary = [
            {"speed": self.v_x, "start": self.arrival_time}]  # 速度の更新予定表
        self.acc_itinerary = kwagrs.get("acc_itinerary")


"""
noiseを早避けするためのacc_itineraryを計算する関数 

a. (1)前処理: acc_itineraryでcurrent_time以後に始まるものは削除
   (2)そもそも最大加速で辿り着くか検討. 無理だったらFalse

b. (1) 続いて、今から一定加速度で加速してnoiseを避ける場合のpathを計算. 
       ただ、この時の加速度は一定範囲のものを選べても良いかもしれない. 
   (2) 経路のvalidate
"""


def calc_early_avoid_acc(noise, current_time, carObj: Cars, ):
    noise_start_time = noise["t"][0]
    noise_end_poisition = noise["x"][1]
    acc_itinerary = [
        accObj for accObj in carObj.acc_itinerary if accObj["start"] <= current_time]

    margin_time_to_noise = noise_start_time - current_time
    max_cover_distance = calc_max_cover_distance(margin_time_to_noise, carObj)
    print(f"delta_t={margin_time_to_noise}, \n 到達可能距離:{max_cover_distance}")
    distance_to_noise_end = noise_end_poisition - carObj.xcor
    if distance_to_noise_end > max_cover_distance["distance"]:
        return False

    # カバーできる場合適当なaccを指定してもらって、それを元に経路を作成.
    # そのためにまずは必要な最低加速度を計算する.
    minimum_required_acc = solve_minimum_required_acc(
        carObj.a_max,  # a_max
        carObj.max_speed,  # v_max
        carObj.v_x,  # v_0
        margin_time_to_noise,  # margin_time_to_noise
        distance_to_noise_end,  # delta_x
    )
    # print(f"最小加速度:{minimum_required_acc}")
    random_param = 0.2
    selected_acc = random_param * carObj.a_max + \
        (1-random_param)*minimum_required_acc

    max_available_acc = calc_max_available_acc()
    acc_itinerary.append({"start": current_time, "acc": selected_acc})

    return acc_itinerary


def calc_max_available_acc():
    return


def calc_max_cover_distance(margin_time_to_noise, carObj: Cars):
    # 最大限加速したら最大スピードに達する場合
    if margin_time_to_noise * carObj.a_max + carObj.v_x > carObj.max_speed:
        delta_t_to_max_speed = (carObj.max_speed - carObj.v_x) / carObj.a_max
        return {"distance": carObj.v_x * delta_t_to_max_speed + 0.5 * carObj.a_max * delta_t_to_max_speed ** 2 +
                carObj.max_speed * (margin_time_to_noise - delta_t_to_max_speed), "v_end": carObj.max_speed}

    # 最大限加速しても最大スピードに乗らない場合
    return {"distance": carObj.v_x * margin_time_to_noise + 0.5 * carObj.a_max * margin_time_to_noise ** 2,
            "v_end": carObj.v_x + margin_time_to_noise * carObj.a_max}


"""
関数: solve_minimum_required_acc

最後 v_maxで終わってもいいけどとりあえずノイズを避けるための最小化速度を計算する
"""


def solve_minimum_required_acc(a_max, v_max, v_0, margin_time_to_noise, delta_x):
    delta_t = margin_time_to_noise
    delta_v = v_max - v_0
    S = delta_x - v_0 * delta_t  # 加速度運動でカバーするべき面積
    # print(f"delta_t:{delta_t}, delta_v:{delta_v}, S:{S}")
    # ベタ踏みしてもv_maxに行かないとき
    if a_max * delta_t < delta_v:
        a_min = 2 * S / delta_t ** 2
        # print(f"CASE A: 1/2*{a_min}*{delta_t}**2={S}")
        return a_min

    # v_maxに届くだけの時間がある場合はv_maxを出す必要があるかどうかで場合わけ.
    # v_maxで走る必要がある場合
    if S > 0.5 * delta_v * delta_t:
        a_min = delta_t ** 2/(2 * v_max * delta_t - S)
        t = {2 * v_max * delta_t - S}
        # print(f"t= {t}")
        # print(f"CASE B: 1/2*{a_min}*{t}**2 + {v_max}*({delta_t}-{t})")

        return delta_t ** 2/(2 * v_max * delta_t - S)

    a_min = 2 * S / delta_t ** 2
    # print(f"CASE C: 1/2*{a_min}*{delta_t}**2={S}")
    return a_min


def test():
    print("============TEST START============")
    current_time = 0
    noise = {"t": [10, 13], "x": [220, 240]}
    acc_itinerary_1 = [{"start": 0, "acc": 3}, {"start": 4, "acc": -1}]
    acc_itinerary_2 = [{"start": 4, "acc": 0}]
    carObj = Cars(
        v_x=20, acc_itinerary=acc_itinerary_1, a_max=3, max_speed=30)
    result = calc_early_avoid_acc(noise, current_time, carObj)
    print(f"result:{result}")


if __name__ == "__main__":
    test()
