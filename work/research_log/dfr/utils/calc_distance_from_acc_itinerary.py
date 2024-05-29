"""
acc_itineraryとteを入れるとteの時点でどこにいるかを計算する関数
acc_itineraryは以下のような形式
{"t_start": 4.0, "acc": 3, "v_0": 20, "t_end": 14}
"""
def calc_distance_from_acc_itinerary(acc_itinerary, te):
    cover_distance = 0
    v_last = acc_itinerary[0]["v_0"] # acc_itineraryの方があっているか確認用
    for idx, acc_info in enumerate(acc_itinerary):
        delta_t = acc_info["t_end"] - acc_info["t_start"]
        v0 = acc_info["v_0"]
        if te < acc_info["t_end"]:
            delta_t = te - acc_info["t_start"]
            cover_distance += v0 * delta_t + 0.5 * acc_info["acc"] * delta_t**2
            return cover_distance

        # print(f"v0={v0}, v_log={v_last}, te={te}, t_end={acc_info['t_end']}")
        if abs(v_last-v0)  > 0.1:
            print(f"id={idx}",acc_itinerary, v0, v_last, acc_info)
            raise ValueError(f"acc_itinerary is wrong, v0={v0}, v_log={v_last}")
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
    acc_itinerary = [{'t_start': 4.0, 'acc': 3, 'v_0': 10, "t_end": 6}, {"t_start": 14, "acc": 0, "v_0": 16.0, "t_end": 20}]
    te = 15
    distance = calc_distance_from_acc_itinerary(acc_itinerary, te)
    print(distance)

if __name__ == "__main__":
    test()        