import copy


def will_collide(x0, v0, t0, decel, xe, te):
    """
    ある瞬間（x0, v0, t0）から全力でteまで減速してもxeに当たってしまうかどうかを判定。当たればTrueを返す.
    """
    delta_t = te - t0
    if delta_t < 0:
        # raise ValueError("delta_t must be positive")
        return False
    # 速度が0にならない場合
    if delta_t < v0**2 / (2 * abs(decel)):
        cover_distance = v0 * delta_t - 0.5 * abs(decel) * delta_t**2
        # print("cover_distance", cover_distance, x0 + cover_distance, xe)
        return x0 + cover_distance > xe
    # 速度が0になる場合
    cover_distance = v0**2 / (2 * abs(decel))
    # print(x0, v0, t0, decel, xe, te, )
    return x0 + cover_distance > xe


def create_earliest_etas(leader_eta, ttc):
    """
    leaderのETAにTTCだけ足したものを返す. 
    """
    leader_eta_list = leader_eta.sort_values(
        by=["x"]).to_dict(orient="records")
    earliest_etas = [{"x": leader_eta["x"], "eta": leader_eta["eta"]+ttc}
                     for leader_eta in leader_eta_list]
    return earliest_etas


def update_acc_itinerary(current_itinerary, new_itinerary):
    """
    itineraryのappend処理をする
    加速度が同じだったら区間を繋げる
    """
    result = copy.deepcopy(current_itinerary)
    # そもそもnew_itineraryの方が前から始まっていたらnew_itineraryで完全にreplaceする.
    if new_itinerary[0]["t_start"] == current_itinerary[0]["t_start"]:
        return new_itinerary
    # 末尾と先頭の加速度が等しい場合は区間を繋げる
    if result[-1]["acc"] == new_itinerary[0]["acc"]:
        result[-1]["t_end"] = new_itinerary[0]["t_end"]
    else:
        result = current_itinerary + new_itinerary
    return result
