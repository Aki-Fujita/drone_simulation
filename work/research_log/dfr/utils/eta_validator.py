import pandas as pd


def validate_with_ttc(eta_reservation_table, car_plans, TTC, **kwargs):
    """
    eta_reservation_table: 全体のスケジュール. DataFrame型を想定. 
    car_plan: 申し込もうとしてるもの. 
    TTC: 空けないといけない車間時間
    この関数はeta_reservation_tableを元に、送られてきたeta_planがValidかどうかを返す. 
    """
    table = eta_reservation_table
    is_valid = True
    car_idx = car_plans[0]["car_idx"]
    df = table[table["car_idx"] == int(car_idx-1)]  # 追い抜きがないので自分より前にいる車 = 一つ前の車
    current_car_position = kwargs.get("car_position", 0)
    current_time = kwargs.get("current_time", 0)

    if df.shape[0] < 1:
        return True
    target_waypoints = [car_plan["x"] for car_plan in car_plans]
    max_eta_by_waypoint = df[df["x"].isin(target_waypoints)].groupby("x")["eta"].max()

    for car_plan_by_x in car_plans[1:]:
        target_waypoint_x = car_plan_by_x["x"]
        if target_waypoint_x < current_car_position:
            continue
        if target_waypoint_x in max_eta_by_waypoint:
            last_entry_time = max_eta_by_waypoint[target_waypoint_x]
            if not (car_plan_by_x["eta"] > last_entry_time + TTC - 0.1):  # 条件に合わない場合
                # if car_idx == 38:
                #     print(car_plan_by_x, last_entry_time)
                return False

    return True


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
    # print(f"itinerary \n{itinerary}")
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
