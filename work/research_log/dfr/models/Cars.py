import sys
sys.path.append("..")
from utils import calc_early_avoid_acc, calc_late_avoid, \
  validate_with_ttc, create_itinerary_from_acc, calc_eta_from_acc, optimizer_for_follower, crt_itinerary_from_a_optimized
import pandas as pd
from .ReservationTable import ReservationTable


class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time", 0)
        self.car_idx = kwagrs.get("index")
        self.v_mean = kwagrs.get("v_mean")
        self.v_max = kwagrs.get("v_max")
        self.a_max = kwagrs.get("a_max")
        self.a_min = kwagrs.get("a_min") # 許容可能な減速度
        self.xcor = 0
        self.xcorList = [0]
        self.timeLog = [self.arrival_time]
        self.v_x = kwagrs.get("v_mean")
        self.itinerary = []  # 自分のETA予定表のこと
        self.acc_itinerary = [{"acc": 0, "t_start": self.arrival_time, "v_0": self.v_x, "t_end": 1e7}]
        if self.a_max == None or self.v_max== None:
            raise ValueError("入力されていない項目があります。")

    def create_desired_eta(self, way_points):
        def calc_eta(way_points):
            estimated_time_of_arrival = way_points["x"] / self.v_mean + self.arrival_time
            return {**way_points, "eta": estimated_time_of_arrival, "car_idx": self.car_idx, "type":"waypoint"}
        way_points_with_eta = list(map(calc_eta, way_points))
        self.itinerary = way_points_with_eta
        return way_points_with_eta

    def get_noise_eta_others(self, table):
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
    
    def select_closest_noise(self, noiseList, current_time):
        required_speeds = []
        for noise in noiseList:
            margin_time = noise["t"][0] - current_time
            noise_end = noise["x"][1]
            required_speed = noise_end / margin_time
            required_speeds.append(required_speed)
            if margin_time < 0:
                raise ValueError("margin time is Negative! Something's wrong!")
        noise_to_avoid = noiseList[required_speeds.index( max(required_speeds))]
        return noise_to_avoid, required_speeds


    def modify_eta(self, noiseList, current_time, table, leader):
        """
        Step1. 各ノイズに対して加速してやり過ごせないかを検討する（x-t線図で言う左下を目指す）. 
        (a) もしもノイズの右端を横切れてかつ、それで他の車にも影響がない場合はそれを新たな経路にする. 
        (b) 上記の達成が不可能な場合はおとなしく左上を目指す. 
        """
        print(f"avoidance by idx={self.car_idx}")
        noise_to_avoid, required_speeds = self.select_closest_noise(noiseList, current_time)
        print(f"Required_speeds:{required_speeds}, V_MAX:{self.v_max}")

        # (a)の場合をまずは検討. 
        temp_acc_itinerary = calc_early_avoid_acc(noise_to_avoid, current_time, self, table)
        if not temp_acc_itinerary:
            temp_acc_itinerary = calc_late_avoid(noise_to_avoid, current_time, self, table, leader)
        
        """この時点でtemp_acc_itineraryは早いものか遅いものが何かしら入っている
            ただし、いずれの場合も未認証. 
        """
        print(f"acc_itinerary:\n{temp_acc_itinerary}")

        ideal_eta = create_itinerary_from_acc(car_obj=self, current_time=current_time, acc_itinerary=temp_acc_itinerary )
        print(ideal_eta)

        # 普通に計画すると前の車にぶつかることがあり得る。
        if validate_with_ttc(table.eta_table, ideal_eta, table.global_params.DESIRED_TTC):
            self.itinerary = ideal_eta
            self.acc_itinerary = temp_acc_itinerary
            return ideal_eta

        # (b)の場合
        print("Value Error 基本的にここには来ないはず")
        noise_to_avoid = noiseList[required_speeds.index(min(required_speeds))]
        raise ValueError("ノイズを避けることができませんでした。")
    
    def add_noise_eta(self, noiselist):
        """
        この関数はノイズをETAに新規追加するだけでPUTは行わないことにする. 
        """
        current_itinerary = self.itinerary
        noise_x_coors = []
        x_in_itinerary = [point["x"] for point in self.itinerary]
        for noise in noiselist:            
            noise_x_coors.append(noise["x"][0])
            noise_x_coors.append(noise["x"][1])
      
        for noise_x_coor in noise_x_coors:
            """
            noise_x_coorsがすでにETAリストにあったらスキップ
            """
            if noise_x_coor in x_in_itinerary:
                continue
            eta_at_noise = calc_eta_from_acc(noise_x_coor, self.acc_itinerary)
            current_itinerary.append({"eta": eta_at_noise, "car_idx": self.car_idx, "type":"noise", "x":noise_x_coor})

    def decide_speed(self, current_time, time_step):
        """
        この関数は自分のacc_itineraryをもとに自分のスピードを決める. 
        """
        acc = self.get_acc_for_time(current_time)
        # print("decide_speed: "self.car_idx, self.acc_itinerary, current_time, acc)
        next_speed = self.v_x + acc * time_step
        self.v_x = next_speed

    def proceed(self, time_step, current_time):
        self.xcor += self.v_x * time_step
        self.xcorList.append(self.xcor)
        self.timeLog.append(current_time)
    
    def get_acc_for_time(self, current_time):
        """
        指定された時刻 t に対して、t 以下で最大の t_start を探し、その時の acc を返す。
        :param data: 辞書のリスト。各辞書は acc, t_start, v_0 のキーを持つ。
        :param t: 指定された時刻。
        :return: 条件を満たす acc の値。該当するものがなければ None を返す。
        """
        # t_start が t 以下の要素のみをフィルタリングし、t_start で降順にソート
        valid_items = sorted([item for item in self.acc_itinerary if item['t_start'] <= current_time], key=lambda x: x['t_start'], reverse=True)
        # 条件を満たす最初の要素の acc を返す
        return valid_items[0]['acc'] if valid_items else None
    

def prepare_test():
    return 


def test():
    print("============TEST START============")
    current_time = 0.5
    noise = {"t": [10, 13], "x": [220, 240]}
    acc_itinerary_1 = [{"t_start": 0, "acc": 3}, {"t_start": 4, "acc": -1}]
    acc_itinerary_2 = [{"t_start": 4, "acc": 0}]
    carObj = Cars(
        mean_speed=20, acc_itinerary=acc_itinerary_1, a_max=3, max_speed=30)

    sample_table = ReservationTable( waypoints=[],
        global_params={"DESIRED_TTC":3}
    )
    carObj.modify_eta(noiseList=[noise], current_time=current_time, table=sample_table)


if __name__ == "__main__":
    test()
