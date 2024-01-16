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

class Hoge:
    def __init__(self):
        self.hensuu = 1
    
    def call(self):
        hoge_func(self)
        

def hoge_func(hogeInstance):
    print(hogeInstance.hensuu)

def test():
    hoge = Hoge()
    hoge.call()


if __name__ == "__main__":
    test()
