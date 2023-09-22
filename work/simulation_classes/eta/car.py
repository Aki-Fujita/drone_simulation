from functions.helly_velocity import helly
import numpy as np
import matplotlib.pyplot as plt


class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time")
        self.index = kwagrs.get("index")
        self.mean_speed = kwagrs.get("mean_speed")
        self.xcor = 0
        self.v_x = kwagrs.get("mean_speed")
        self.helly_params = kwagrs.get("helly_params")
        self.target_speed = 0
        self.xcorList = []
        self.v_xList = []
        # この辺はETAのCONTROLに必要になる
        self.group_id = kwagrs.get("group_id")
        self.order_in_grouop = kwagrs.get("order_in_group")
        self.speed_profile = []

    def create_desired_list(self, way_points):

        def calc_eta(way_point):
            estimated_time_of_arrival = way_point["x"] / self.mean_speed + self.arrival_time
            return {**way_point, "eta": estimated_time_of_arrival, "car_idx": self.index,
                    "group_id": self.group_id, "order_in_group": self.order_in_grouop}
        way_points_with_eta = list(map(calc_eta, way_points))

        return way_points_with_eta

    def decide_speed(self, time_step, delta_x, delta_v):
        helly_speed = helly(delta_x, delta_v, time_step, self.v_x, self.helly_params)

        def decide_next_speed(helly, target):
            return helly

        next_speed = decide_next_speed(helly_speed, self.target_speed)
        self.v_x = next_speed
        return next_speed

    def proceed(self, **kwargs):
        self.xcor += kwargs.get("time_step") * self.v_x

    def record(self):
        self.xcorList.append(self.xcor)
        self.v_xList.append(self.v_x)

    def plot_speed_profile(self, partition_num=200, v_lim=40):
        if self.speed_profile == []:
            print("speed_profileが未計算です")
            return
        time_to_exit = 0
        for phase in self.speed_profile:
            time_to_exit += phase["duration"]
        times = np.linspace(0, int(time_to_exit), partition_num)
        v_0 = self.speed_profile[0]["initial_speed"]

        def calc_speed_by_time(speed_profile, t, v_0):
            thresh_1 = speed_profile[0]["duration"]
            thresh_2 = thresh_1 + speed_profile[1]["duration"]
            v_after_phase_1 = v_0 + speed_profile[0]["ACC"] * speed_profile[0]["duration"]
            v_after_phase_2 = v_after_phase_1 + speed_profile[1]["ACC"] * speed_profile[1]["duration"]
            if t <= thresh_1:
                return v_0 + speed_profile[0]["ACC"] * t
            if t <= thresh_2:
                return v_after_phase_1 + (t - thresh_1) * speed_profile[1]["ACC"]
            return v_after_phase_2 + (t - thresh_2) * speed_profile[2]["ACC"]

        v_xlist = [calc_speed_by_time(self.speed_profile, time, v_0) for time in times]

        # グラフのタイトルや軸ラベルの設定
        plt.plot(times, v_xlist)
        plt.ylim(0, v_lim)
        plt.title("Speed Profile")
        plt.xlabel("Time")
        plt.ylabel("v_x")
        plt.grid()
