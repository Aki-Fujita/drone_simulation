def merge_acc_itinerary_by_time(pre_itinerary, new_itinerary):
    """
    2つのacc_itineraryを時間順にマージする. 
    """
    # print("pre_itinerary: ", pre_itinerary)
    # print("new_itinerary: ", new_itinerary)
    merged_itinerary = []
    new_itinerary_start = new_itinerary[0]["t_start"]

    # new_itineraryが完全に新しく作られている場合. 
    if new_itinerary_start <= pre_itinerary[0]["t_start"]:
        return new_itinerary

    # 完全に後のものが入ってきたらそのまま追加
    if pre_itinerary[-1]["t_end"] <= new_itinerary[0]["t_start"]:
        result[-1]["t_end"] = new_itinerary[0]["t_start"]
        result = pre_itinerary + new_itinerary
        return result
    
    # ここに来る場合、pre_itineraryの終わりよりもnew_itineraryの始まりが早い
    # => つまり preの途中でnewを合成したものがmergedになる. 

    for pre_segment in pre_itinerary:
        # new_itinerary_startまではpre_itineraryの情報をそのまま使う
        if pre_segment["t_end"] < new_itinerary_start:
            merged_itinerary.append(pre_segment) 
            continue
        
        elif pre_segment["t_end"] >= new_itinerary_start:
            fixed_pre_item = {**pre_segment, "t_end": new_itinerary_start}
            merged_itinerary.append(fixed_pre_item)
            break
        
    merged_itinerary += new_itinerary

    return merged_itinerary

    