import copy


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
