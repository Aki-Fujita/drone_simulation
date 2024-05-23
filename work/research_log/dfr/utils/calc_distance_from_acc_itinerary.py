"""
acc_itineraryとteを入れるとteの時点でどこにいるかを計算する関数
acc_itineraryは以下のような形式
{"t_start": 4.0, "acc": 3, "v_0": 20, "t_end": 14}
"""
def calc_distance_from_acc_itinerary(acc_itinerary, te):
    cover_distance = 0
    v_log = acc_itinerary[0]["v_0"] # acc_itineraryの方があっているか確認用
    for idx, acc_info in enumerate(acc_itinerary):
        delta_t = acc_info["t_end"] - acc_info["t_start"]
        if te < acc_info["t_end"]:
            delta_t = te - acc_info["t_start"]
        v0 = acc_info["v_0"]
        if abs(v_log-v0)  > 0.1:
            print(acc_itinerary)
            raise ValueError(f"acc_itinerary is wrong, v0={v0}, v_log={v_log}")
        cover_distance += v0 * delta_t + 0.5 * acc_info["acc"] * delta_t**2
        print(f"i={idx}, cover_distance={cover_distance}")

        # 次の区間に行く前にv_logを更新
        v_log += delta_t * acc_info["acc"]
    return cover_distance

def test():
    print("TEST START: calc_distance_from_acc_itinerary.py")
    acc_itinerary = [{'t_start': 4.0, 'acc': 3, 'v_0': 10, "t_end": 6}, {"t_start": 14, "acc": 0, "v_0": 16.0, "t_end": 20}]
    te = 15
    distance = calc_distance_from_acc_itinerary(acc_itinerary, te)
    print(distance)

if __name__ == "__main__":
    test()        