import pandas as pd


def validate_with_ttc(eta_reservation_table, car_plans, TTC):
    """
    eta_reservation_table: 全体のスケジュール. DataFrame型を想定. 
    car_plan: 申し込もうとしてるもの. 
    TTC: 空けないといけない車間時間
    この関数はeta_reservation_tableを元に、送られてきたeta_planがValidかどうかを返す. 
    """
    table = eta_reservation_table
    is_valid = True
    car_idx = car_plans[0]["car_idx"]
    df = table[table["car_idx"] < car_idx]  # 追い抜きがないので自分より前にいる車 = indexが若い車

    if df.shape[0] < 1:
        return True

    for idx, car_plan_by_x in enumerate(car_plans):
        if idx == 0:
            continue
        target_waypoint_x = car_plan_by_x["x"]
        filtered_df = df[df["x"] == target_waypoint_x]
        # print(f"len(wpts)={len(filtered_df)}")
        last_entry_time = filtered_df["eta"].max()
        if car_plan_by_x["eta"] > last_entry_time + TTC:
            continue
        else:
            print("eta_validator.py:  INVALID")
            print(table[table["car_idx"] == car_idx-1])
            # print(ERT[ERT["x"] < 150])
            print(car_plan_by_x)
            print(last_entry_time, TTC, car_idx)
            # print(f"wp_x={car_plan_by_x["x"]}, TTC={car_plan_by_x["eta"] - last_entry_time}")
            is_valid = False
            break
    return is_valid


def create_sample_itinerary():
    WAYPOINTS_NUM = 10
    arrival_time = 0.4
    car_idx = 0
    WAYPOINTS = [{"waypoint_idx": i, "x": 1000 /
                  WAYPOINTS_NUM * (i)} for i in range(WAYPOINTS_NUM+1)]
    itinerary = []
    for waypoint in WAYPOINTS:
        estimated_time_of_arrival = waypoint["x"] / 20 + arrival_time
        itinerary.append(
            {**waypoint,  "eta": estimated_time_of_arrival, "car_idx": car_idx})
    print(f"itinerary \n{itinerary}")
    return itinerary


def create_sample_df():
    WAYPOINTS_NUM = 10
    arrival_times = [0.4, 4, 9, 11, 30, 40, 50, 60, 70, 80]
    WAYPOINTS = [{"waypoint_idx": i, "x": 1000 /
                  WAYPOINTS_NUM * (i)} for i in range(WAYPOINTS_NUM+1)]
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
    validate_with_ttc(df, itinerary, 2)


if __name__ == "__main__":
    test()
