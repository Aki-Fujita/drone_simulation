"""
acc_itineraryとteを入れるとteの時点でどこにいるかを計算する関数
acc_itineraryは以下のような形式
{"t_start": 4.0, "acc": 3, "v_0": 20, "t_end": 14, "x_start": 10}
"""


def calc_distance_from_acc_itinerary(acc_itinerary, te):
    cover_distance = 0
    v_last = acc_itinerary[0]["v_0"]  # acc_itineraryの方があっているか確認用
    for idx, acc_info in enumerate(acc_itinerary):
        delta_t = acc_info["t_end"] - acc_info["t_start"]
        v0 = acc_info["v_0"]
        if te < acc_info["t_end"]:
            delta_t = te - acc_info["t_start"]
            cover_distance += v0 * delta_t + 0.5 * acc_info["acc"] * delta_t**2
            return cover_distance

        # print(f"v0={v0}, v_log={v_last}, te={te}, t_end={acc_info['t_end']}")
        # ここのthresholdはちゃんと考えないといけない。simulation時間のtickが0.2だとすると、加速度3の区間では最大0.3（3 * 0.2 /2）のずれが生じる
        # ETAだけで動くわけではなく, DAAも考慮するようにしたのでacc_itineraryが狂うこともあるため、以下のコードは無効にした. 
        # if abs(v_last-v0) > 0.299:
        #     print(f"id={idx}", acc_itinerary, v0, v_last, acc_info)
        #     raise ValueError(f"acc_itinerary is wrong, v0={
        #                      v0}, v_log={v_last}, acc_itinerary={acc_itinerary}, te={te}")
        cover_distance += v0 * delta_t + 0.5 * acc_info["acc"] * delta_t**2
        # print(f"i={idx}, cover_distance={cover_distance}")
        v_last += acc_info["acc"] * delta_t

    # 最後の区間が終わった後の時刻を指定されたら、最後の区間のt_end時点から等速で進んだものとして計算.
    if te > acc_itinerary[-1]["t_end"]:
        delta_t = te - acc_itinerary[-1]["t_end"]
        cover_distance += v_last * delta_t
    return cover_distance


def test():
    print("TEST START: calc_distance_from_acc_itinerary.py")
    acc_itinerary = [{'t_start': 15.0, 'acc': -3, 'v_0': 20, 't_end': 15.5}, {'t_start': 15.5, 'acc': 0, 'v_0': 18.5, 't_end': 25.79054054054054}, {'t_start': 25.79054054054054, 'acc': -3, 'v_0': 18.5, 't_end': 27.79054054054054}, {'t_start': 27.79054054054054, 'acc': 0, 'v_0': 12.5, 't_end': 33.31054054054054}, {'t_start': 33.31054054054054, 'acc': 2, 'v_0': 12.5, 't_end': 34.06054054054054}, {'t_start': 34.06054054054054, 'acc': 0, 'v_0': 14.0, 't_end': 47.6364333976834}, {'t_start': 47.6364333976834, 'acc': 2, 'v_0': 14.0, 't_end': 52.6364333976834}, {'t_start': 52.6364333976834, 'acc': 0, 'v_0': 24.0, 't_end': 52.99357625482625},
                     {'t_start': 52.99357625482625, 'acc': 2, 'v_0': 24.0, 't_end': 53.24357625482625}, {'t_start': 53.24357625482625, 'acc': 0, 'v_0': 24.5, 't_end': 53.25882923101673}, {'t_start': 53.25882923101673, 'acc': 2, 'v_0': 24.5, 't_end': 56.25882923101673}, {'t_start': 56.25882923101673, 'acc': 0, 'v_0': 30.5, 't_end': 56.56464039476795}, {'t_start': 56.56464039476795, 'acc': 2, 'v_0': 30.5, 't_end': 57.31464039476795}, {'t_start': 57.31464039476795, 'acc': 0, 'v_0': 32.0, 't_end': 59.6498789265646}, {'t_start': 59.6498789265646, 'acc': 2, 'v_0': 32.0, 't_end': 59.8998789265646}, {'t_start': 59.8998789265646, 'acc': 0, 'v_0': 32.5, 't_end': 10000000.0}]
    te = 50
    distance = calc_distance_from_acc_itinerary(acc_itinerary, te)
    print(distance)


if __name__ == "__main__":
    test()
