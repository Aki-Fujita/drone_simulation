from research_log.eta.utils import find_delta_v_list, find_delta_x_list
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("../")


class ETASimulation:
    def __init__(self, **kwargs):
        self.CARS = kwargs.get("CARS")
        self.cwp_table = kwargs.get("CWPTable")
        self.simulation_params = kwargs.get("simulation_params")

    def conduct_simulation(self):
        sp = self.simulation_params
        SIMULATION_STEPS = int(sp["TOTAL_TIME"] / sp["TIME_STEP"])

        for i in range(SIMULATION_STEPS):
            t = i * sp["TIME_STEP"]
            delta_x_list = find_delta_x_list(self.CARS)
            delta_v_list = find_delta_v_list(self.CARS)

            for idx, car in enumerate(self.CARS):
                # この時間に到着する車がいれば打刻する
                if car.arrival_time >= t and car.arrival_time < t + sp["TIME_STEP"]:
                    print("idx={0}, エントランス到着時刻={1}".format(idx, car.arrival_time))
                    desired_list = car.create_desired_list(self.cwp_table.waypoints)
                    # print(desired_list)
                    is_valid = self.cwp_table.validate(desired_list)
                    if is_valid:
                        self.cwp_table.register(desired_list)
                    else:
                        calibration_info = {"desired_list": desired_list, "enter_speed": car.mean_speed,
                                            "max_acc": car.helly_params["max_accel"], "max_dec": car.helly_params["rear_brake_acc"]}
                        calibrated_list, speed_profile = self.cwp_table.calibrate_list(**calibration_info)
                        # print(calibrated_list)
                        self.cwp_table.register(calibrated_list)
                        car.speed_profile = speed_profile
                    continue
                # そうでない車は普通に進む
                # delta_x = delta_x_list[idx]
                # delta_v = delta_v_list[idx]
                # speed = car.decide_speed(TIME_STEP, delta_x, delta_v)
                # delta_xとdelta_vの計算を移動前にやっているので実質pararellになっている
                # car.proceed(time_step=TIME_STEP)
                # car.record()

    # 整流率の計算
    def calc_rectification(self, exit=None):
        def calc_interval_deviation(num_list):
            intervals = [num_list[i + 1] - num_list[i] for i in range(len(num_list) - 1)]
            mean_interval = sum(intervals) / len(intervals)
            sd = (sum([(interval - mean_interval) ** 2 for interval in intervals]) / len(intervals)) ** 0.5
            return sd

        df = self.cwp_table.waypoint_table
        orifith_exit = self.cwp_table.global_params.ORIFITH_EXIT_INDEX
        arrival_time_list = df[df["x"] == 0]["eta"].tolist()
        exit_time_list = df[df["waypoint_idx"] == orifith_exit]["eta"].tolist()
        arrival_time_sd = calc_interval_deviation(arrival_time_list)
        exit_time_sd = calc_interval_deviation(exit_time_list)
        return exit_time_sd / arrival_time_sd
