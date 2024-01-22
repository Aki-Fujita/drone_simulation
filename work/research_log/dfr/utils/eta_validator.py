import pandas as pd

def validate_with_ttc(eta_reservation_table, car_plan, TTC):
    """
    eta_reservation_table: 全体のスケジュール. DataFrame型を想定. 
    car_plan: 申し込もうとしてるもの. 
    TTC: 空けないといけない車間時間
    この関数はeta_reservation_tableを元に、送られてきたeta_planがValidかどうかを返す. 
    """
    df = eta_reservation_table
    is_valid = True
    car_idx = car_plan[0]["car_idx"]
    df = df[df["car_idx"] < car_idx]  # 追い抜きがないので自分より前にいる車 = indexが若い車
    print(f"Len(filtered_df) = {len(df)}")

    if df.shape[0] < 1:
        return True

    for idx, waypoint_info in enumerate(car_plan):
        print(f"idx={idx}")
        if idx == 0:
            continue

        target_waypoint = waypoint_info["waypoint_idx"]
        filtered_df = df[df["waypoint_idx"] == target_waypoint]
        print(f"len(wpts)={len(filtered_df)}")
        last_entry_time = filtered_df["eta"].max()
        if waypoint_info["eta"] > last_entry_time + TTC:
            continue
        else:
            print(f"wp_idx={idx}")
            print("INVALID")
            is_valid = False
            break
    return is_valid

def create_sample_itinerary():
    WAYPOINTS_NUM = 10
    arrival_time = 0.4
    car_idx = 0
    WAYPOINTS = [{"waypoint_idx": i, "x": 1000 / WAYPOINTS_NUM * (i)} for i in range(WAYPOINTS_NUM+1)]
    itinerary = []
    for waypoint in WAYPOINTS:
        estimated_time_of_arrival = waypoint["x"] / 20 + arrival_time
        itinerary.append({**waypoint,  "eta": estimated_time_of_arrival, "car_idx":car_idx})
    print(f"itinerary \n{itinerary}")
    return itinerary

def create_sample_df():
    WAYPOINTS_NUM = 10
    arrival_times=[0.4, 4, 9, 11, 30, 40, 50, 60, 70, 80]
    WAYPOINTS = [{"waypoint_idx": i, "x": 1000 / WAYPOINTS_NUM * (i)} for i in range(WAYPOINTS_NUM+1)]
    etas = []
    for idx, arrival_time in enumerate(arrival_times):
        """
        ここからwaypointsに対してetaを計算
        """
        def calc_eta(way_points):
            estimated_time_of_arrival = way_points["x"] / 20 + arrival_time
            return {**way_points, "eta": estimated_time_of_arrival, "car_idx": idx}
    
        way_points_with_eta = list(map(calc_eta, WAYPOINTS))
        print(way_points_with_eta)
        etas += way_points_with_eta
    print("====ETAs======")
    # print(etas)
    print("=======")
    return pd.DataFrame(etas)

def test():
    df = create_sample_df()
    itinerary = create_sample_itinerary()
    print(df)
    validate_with_ttc(df, itinerary,2 )


if __name__ == "__main__":
    test()
