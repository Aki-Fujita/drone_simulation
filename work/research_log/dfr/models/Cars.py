import sys
sys.path.append("..")
from utils import calc_early_avoid_acc, validate_with_ttc, create_eta_from_acc
import pandas as pd
from ReservationTable import ReservationTable


class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time", 0)
        self.index = kwagrs.get("index")
        self.mean_speed = kwagrs.get("mean_speed")
        self.max_speed = kwagrs.get("max_speed")
        self.a_max = kwagrs.get("a_max")
        self.xcor = 0
        self.xcorList = []
        self.timeLog = []
        self.v_x = kwagrs.get("mean_speed")
        self.itinerary = []  # 自分のETA予定表のこと
        self.speed_itinerary = [
            {"speed": self.v_x, "start": self.arrival_time}]  # 速度の更新予定表
        self.acc_itinerary = [{"acc": 0, "start": self.arrival_time, "v_0":self.v_x}]

    def create_desired_eta(self, way_points):
        def calc_eta(way_points):
            estimated_time_of_arrival = way_points["x"] / \
                self.mean_speed + self.arrival_time
            return {**way_points, "eta": estimated_time_of_arrival, "car_idx": self.index}
        way_points_with_eta = list(map(calc_eta, way_points))
        self.itinerary = way_points_with_eta

        return way_points_with_eta

    def consider_others(self, table):
        """
        Step1. 自分の一つ前の車のETA情報を取得. 
        Step2. その車とTTCを空けるようにコースを決める
        """
        print(f"consideration by idx={self.index}")

        df = table.eta_table
        TTC = table.global_params.DESIRED_TTC
        previous_plan = self.itinerary.copy()
        ETAs_for_leading_car = df[df["car_idx"] == int(self.index-1)]
        new_eta = []
        for idx, row in enumerate(ETAs_for_leading_car):
            if row["eta"] + TTC > previous_plan[idx]["eta"]:
                previous_plan[idx]["eta"] = row["eta"] + TTC
            new_eta.append(previous_plan[idx])
        self.itinerary = new_eta
        return new_eta

    def avoid_noise(self, noiseList, current_time, table):
        """
        Step1. 各ノイズに対して加速してやり過ごせないかを検討する（x-t線図で言う左下を目指す）. 
        (a) もしもノイズの右端を横切れてかつ、それで他の車にも影響がない場合はそれを新たな経路にする. 
        (b) 上記の達成が不可能な場合はおとなしく左上を目指す. 
        """
        print(f"avoidance by idx={self.index}")
        required_speeds = []
        reservation_table = table.eta_table
        TTC = table.global_params.DESIRED_TTC

        for noise in noiseList:
            margin_time = noise["t"][0] - current_time
            noise_end = noise["x"][1]
            required_speed = noise_end / margin_time
            required_speeds.append(required_speed)
            if margin_time < 0:
                raise ValueError("margin time is Negative! Something's wrong!")

        # (a)の場合
        if max(required_speeds) <= self.max_speed:
            print("trying accel avoid")
            noise_to_avoid = noiseList[required_speeds.index( max(required_speeds))]
            temp_acc_itinerary = calc_early_avoid_acc(noise_to_avoid, current_time, self)
            ideal_eta = create_eta_from_acc(
                car_obj=self, current_time=current_time
            )
            if validate_with_ttc(reservation_table, ideal_eta, TTC):
                self.itinerary = ideal_eta
                self.acc_itinerary = temp_acc_itinerary
                return ideal_eta

        # (b)の場合
        print("遅い側で避ける")
        noise_to_avoid = noiseList[required_speeds.index(min(required_speeds))]
        ideal_eta = self.calc_decel_eta(noise_to_avoid, current_time)
        self.itinerary = ideal_eta

        return ideal_eta

    # ノイズを避けるために加速する経路.
    def calc_accel_eta(self, noise_to_avoid, current_time):
        """
        (a) すでに通り過ぎたところのetaは変えない
        (b) これから行く場所に対して、それがノイズより手前なら最大スピードで超えに行く
        (c) ノイズより後ならノイズまで最大速度で走ってその後通常スピードへ
        """
        previous_plan = self.itinerary
        new_eta = []
        noise_end = noise_to_avoid["x"][1]
        for _, row in enumerate(previous_plan):
            if self.xcor >= row["x"]:  # その場所を通り過ぎていたら
                new_eta.append(row)
            elif self.xcor < row["x"] and row["x"] <= noise_end:
                next_waypoint_eta = current_time + \
                    (row["x"] - self.xcor)/self.max_speed
                new_plan = {**row, "eta": next_waypoint_eta}
                new_eta.append(new_plan)
            elif self.xcor < row["x"] and row["x"] > noise_end:
                next_waypoint_eta = current_time + \
                    (noise_end - self.xcor)/self.max_speed + \
                    (row["x"] - noise_end)/self.mean_speed
                new_plan = {**row, "eta": next_waypoint_eta}
                new_eta.append(new_plan)
            else:
                print(f"car_id: {self.index}, waypoint={row}")
                raise ValueError("こんなケースはないはず！")

        return new_eta

    # ノイズを避けるために減速する経路.
    def calc_decel_eta(self, noise_to_avoid, current_time):
        """
        まずはspeed_itineraryを更新する. 
        (1) current_timeより昔の履歴は残す
        (2) current_time以降のものはdecel_etaで計算されたものを適用.
        """
        new_speed_itinerary = [
            item for item in self.speed_itinerary if item["start"] < current_time]

        """
        (a) すでに通り過ぎたところのetaは変えない
        (b) これから行く場所に対して、それがノイズより手前ならノイズの左上を横切るためのスピードで走った時のETAを記録
        (c) ノイズより後ならノイズまで所定速度で走ってその後通常スピードへ
        """
        previous_plan = self.itinerary

        new_eta = []
        noise_start = noise_to_avoid["x"][0]
        ideal_speed = 0
        noise_end_time = noise_to_avoid["t"][1]

        for _, row in enumerate(previous_plan):
            if ideal_speed < 0:
                raise ValueError("ideal_speed is Negative!!")
            if self.xcor >= row["x"]:  # その場所を通り過ぎていたら
                new_eta.append(row)
                continue
            elif row["x"] <= noise_start:
                ideal_speed = (noise_start - self.xcor) / \
                    (noise_end_time-current_time)
                next_waypoint_eta = current_time + \
                    (row["x"] - self.xcor)/ideal_speed
                new_plan = {**row, "eta": next_waypoint_eta}
                new_eta.append(new_plan)
                new_speed_itinerary.append({
                    "speed": ideal_speed, "start": current_time
                })
                continue
            elif row["x"] > noise_start:  # waypointがノイズstartよりあとの場合noiseを避けた後普通に走る. ただこれだと当たってしまうのか！
                next_waypoint_eta = current_time + \
                    (noise_start - self.xcor)/ideal_speed + \
                    (row["x"] - noise_start) / self.mean_speed
                new_plan = {**row, "eta": next_waypoint_eta}
                new_eta.append(new_plan)
                new_speed_itinerary.append({
                    "speed": ideal_speed, "start": current_time,
                    "speed": self.mean_speed, "start": current_time +
                    (noise_start - self.xcor)/ideal_speed
                })
                continue

            else:
                print(
                    f"car_id: {self.index}, waypoint={row}, noise_info={noise_to_avoid}")
                raise ValueError("こんなケースはないはず！")

        """
        最後にnew_speed_itineraryの重複を削除する
        """
        speed_itinerary = []
        for item in new_speed_itinerary:
            continue
        return new_eta

    def decide_speed(self, current_time):
        """
        この関数は自分のETA計画表をもとに自分のスピードを決める. 
        """
        future_waypoints = [
            w for w in self.itinerary if w['x'] - self.xcor > 0]
        closest_waypoint = min(
            future_waypoints, key=lambda w: abs(w['x'] - self.xcor))
        distance = closest_waypoint['x'] - self.xcor
        time_difference = closest_waypoint['eta'] - current_time
        if time_difference != 0:
            speed = distance / time_difference
        else:
            speed = self.mean_speed  # 時間差が0の場合、速度は0とする

        self.v_x = speed

    def proceed(self, time_step, current_time):
        self.xcor += self.v_x * time_step
        self.xcorList.append(self.xcor)
        self.timeLog.append(current_time)
    

def test():
    print("============TEST START============")
    current_time = 0.5
    noise = {"t": [10, 13], "x": [220, 240]}
    acc_itinerary_1 = [{"start": 0, "acc": 3}, {"start": 4, "acc": -1}]
    acc_itinerary_2 = [{"start": 4, "acc": 0}]
    carObj = Cars(
        mean_speed=20, acc_itinerary=acc_itinerary_1, a_max=3, max_speed=30)

    sample_table = ReservationTable( waypoints=[],
        global_params={"DESIRED_TTC":3}
    )
    carObj.avoid_noise(noiseList=[noise], current_time=current_time, table=sample_table)


if __name__ == "__main__":
    test()
