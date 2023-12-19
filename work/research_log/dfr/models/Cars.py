import pandas as pd
import sys
sys.path.append("../")
from utils import validate_with_ttc


class Cars:
    def __init__(self, **kwagrs):
        self.eta_table = []
        self.arrival_time = kwagrs.get("arrival_time")
        self.index = kwagrs.get("index")
        self.mean_speed = kwagrs.get("mean_speed")
        self.xcor = 0
        self.v_x = kwagrs.get("mean_speed")

    def create_desired_eta(self, way_points):

        def calc_eta(way_points):
            estimated_time_of_arrival = way_points["x"] / \
                self.mean_speed + self.arrival_time
            return {**way_points, "eta": estimated_time_of_arrival, "car_idx": self.index}
        way_points_with_eta = list(map(calc_eta, way_points))
        self.eta_table = way_points_with_eta

        return way_points_with_eta

    def consider_others(self, table):
        """
        Step1. 自分の一つ前の車のETA情報を取得. 
        Step2. その車とTTCを空けるようにコースを決める
        """
        df = table.eta_table
        TTC = table.global_params.DESIRED_TTC
        previous_plan = self.eta_table
        ETAs_for_reference = df[df["car_idx"] == int(self.index-1)]
        new_eta = []
        for idx, row in enumerate(ETAs_for_reference):
            if row["eta"] + TTC > previous_plan[idx]["eta"]:
                previous_plan[idx]["eta"] = row["eta"] + TTC
            new_eta.append(previous_plan[idx])

        return

    def avoid_noise(self, noiseList, current_time):
        """
        Step1. 各ノイズに対して加速してやり過ごせないかを検討する（x-t線図で言う左下を目指す）. 
        (a) もしもノイズの左端を横切れてかつ、それで他の車にも影響がない場合はそれを新たな経路にする. 
        """
        necessary_speeds = []
        for noise in noiseList:
            margin_time = noise["t"][0] - current_time
            noise_end = noise["x"][1]
            necessary_speeds.append(noise_end / margin_time)
        if max(necessary_speeds) <= self.max_speed:
            ideal_eta = 0# 今この瞬間から全速力を出した場合のETA
            if 

            
        


        return
