from functions.helly_velocity import helly
import numpy as np
import matplotlib.pyplot as plt


class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time")
        self.index = kwagrs.get("index")
        self.mean_speeed = kwagrs.get("mean_speed")
        self.xcor = 0
        self.v_x = kwagrs.get("mean_speed")
        self.helly_params = kwagrs.get("helly_params")
        self.target_speed = 0
        self.xcorList = []
        self.v_xList = []
        # この辺はETAのCONTROLに必要になる
        self.group_id = kwagrs.get("group_id")
        self.order_in_grouop = kwagrs.get("order_in_group")

    def create_desired_list(self, way_points):

        def calc_eta(way_point):
            estimated_time_of_arrival = way_point["x"] / self.mean_speeed + self.arrival_time
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
