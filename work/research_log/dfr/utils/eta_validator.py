def validate_with_ttc(eta_reservation_table, eta_plan, TTC):
    """
    eta_reservation_table: 全体のスケジュール. DataFrame型を想定. 
    eta_plan: 申し込もうとしてるもの. 
    TTC: 空けないといけない車間時間

    この関数はeta_reservation_tableを元に、送られてきたeta_planがValidかどうかを返す. 

    """
    df = eta_reservation_table
    if df.shape[0] < 1:
        return True

    is_valid = True
    car_idx = eta_plan[0]["car_idx"]
    df = df[df["car_idx"] != car_idx]
    for idx, waypoint_info in enumerate(eta_plan):
        if idx == 0:
            continue

        target_waypoint = waypoint_info["waypoint_idx"]
        filtered_df = df[df["waypoint_idx"] == target_waypoint]
        last_entry_time = filtered_df["eta"].max()
        if waypoint_info["eta"] > last_entry_time + TTC:
            continue
        else:
            is_valid = False
            break
    return is_valid


def test():
    print("hello")


if __name__ == "__main__":
    test()
