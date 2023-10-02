import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass
from .PathPlanner import PathPlanner


@dataclass
class GlobalParams:
    WINDOW_SIZE: float
    START_TIME: float
    DESIRED_TTC: float
    DESIRED_SPEED: float
    ORIFITH_EXIT_INDEX: int
    V_MAX: float  # 整流区間内で出して良い最大スピード


class CWPTable:
    def __init__(self, **kwargs) -> None:
        self.waypoints = kwargs.get("waypoints")
        self.algorithm = kwargs.get("algorithm", "KISS")
        self.global_params = GlobalParams(**kwargs.get("global_params"))
        self.waypoint_table = pd.DataFrame([])
        orifith_end_index = self.global_params.ORIFITH_EXIT_INDEX
        self.ORIFITH_LENGTH = self.waypoints[orifith_end_index]["x"]
        print(self.global_params.WINDOW_SIZE)

    def validate(self, waypoints_with_eta):
        if self.algorithm == "KISS":
            result = self.validate_with_kiss(waypoints_with_eta)
            return result
        elif self.algorithm == "CONTROLLED":
            return False

        return False

    def validate_with_kiss(self, waypoints_with_eta):
        df = self.waypoint_table
        if df.shape[0] < 1:
            print("初回")
            return True

        is_valid = True
        for idx, waypoint_info in enumerate(waypoints_with_eta):
            if idx == 0:
                continue

            target_waypoint = waypoint_info["waypoint_idx"]
            filtered_df = df[df["waypoint_idx"] == target_waypoint]
            last_entry_time = filtered_df["eta"].max()
            if waypoint_info["eta"] > last_entry_time + self.global_params.DESIRED_TTC:
                continue
            else:
                is_valid = False
                break
        return is_valid

    def validate_with_controlled(self, waypoints_with_eta):
        is_valid = True
        return is_valid

    def register(self, waypoints_with_eta):
        df_to_add = pd.DataFrame(waypoints_with_eta)
        combined_df = pd.concat([self.waypoint_table, df_to_add])
        self.waypoint_table = combined_df

    def calibrate_list(self, **kwargs):
        df = self.waypoint_table
        calibrated_list = []
        if "desired_list" not in kwargs:
            raise ValueError("キーワード引数 'desired_list' が必要です。")

        waypoints_with_eta = kwargs.get("desired_list")

        if self.algorithm == "KISS":
            for idx, waypoint_info in enumerate(waypoints_with_eta):
                if idx == 0:
                    calibrated_list.append(waypoint_info)
                    continue
                target_waypoint_idx = waypoint_info["waypoint_idx"]
                filtered_df = df[df["waypoint_idx"] == target_waypoint_idx]
                # ここから時間の比較
                last_entry_time = filtered_df["eta"].max()
                desired_entry_time = waypoint_info["eta"]
                if (desired_entry_time < last_entry_time + self.global_params.DESIRED_TTC):
                    entry_time = last_entry_time + self.global_params.DESIRED_TTC
                else:
                    entry_time = desired_entry_time
                calibrated_list.append({**waypoint_info, "eta": entry_time})
            speed_profile = []
            return calibrated_list, speed_profile

        if self.algorithm == "CONTROLLED":
            enter_speed = kwargs.get("enter_speed")
            max_acc = kwargs.get("max_acc", 0.5)
            max_dec = kwargs.get("max_dec", 0.4)
            car_spec = {"enter_speed": enter_speed, "max_acc": max_acc, "max_dec": max_dec, "v_max": self.global_params.V_MAX}
            calibrated_list.append(waypoints_with_eta[0])
            ideal_params_at_end = self.calc_ideal_params_at_end(waypoints_with_eta)
            initial_params = {"time": waypoints_with_eta[0]["eta"], "speed": enter_speed}
            pathPlanner = PathPlanner(
                car_spec=car_spec, initial_params=initial_params, ideal_params_at_end=ideal_params_at_end,
                COURSE_LENGTH=self.ORIFITH_LENGTH
            )
            speed_profile = pathPlanner.solve_path()
            # print(speed_profile)
            # 続いてこのspeed_profileをもとに各場所へのETAを計算する。
            calibrated_waypoints = self.convert_profile_to_eta(speed_profile, waypoints_with_eta)
            # print(calibrated_waypoints)

            return calibrated_waypoints, speed_profile

    def calc_arrival_time(self, speed_profile, distance_from_start):
        d = distance_from_start
        # print(speed_profile)
        if len(speed_profile) == 3:
            phase_1 = speed_profile[0]
            phase_2 = speed_profile[1]
            phase_3 = speed_profile[2]
            v_0 = phase_1["initial_speed"]

            if phase_1["ACC"] == 0:
                # CACの場合
                end_of_phase_1 = phase_1["duration"] * v_0
                end_of_phase_2 = phase_2["ACC"] * phase_2["duration"] ** 2 * 0.5 + v_0 * phase_2["duration"] + end_of_phase_1
                if d <= end_of_phase_1:
                    return d / v_0
                if d <= end_of_phase_2:
                    delta = d - end_of_phase_1
                    return ((v_0 ** 2 + 2 * phase_2["ACC"] * delta)**0.5 - v_0) / phase_2["ACC"] + phase_1["duration"]
                return (d - end_of_phase_2) / phase_3["initial_speed"] + phase_1["duration"] + phase_2["duration"]

            else:
                # ACDまたはDCAの場合
                end_of_phase_1 = phase_1["ACC"] * phase_1["duration"]**2 * 0.5 + phase_1["duration"] * v_0
                end_of_phase_2 = end_of_phase_1 + phase_2["duration"] * phase_2["initial_speed"]
                end_of_phase_3 = phase_3["initial_speed"] * phase_3["duration"] + 0.5 * phase_3["ACC"] * phase_3["duration"]**2 + end_of_phase_2

                if d <= end_of_phase_1:
                    return ((v_0 ** 2 + 2 * phase_1["ACC"] * d)**0.5 - v_0) / phase_1["ACC"]
                if d <= end_of_phase_2:
                    return (d - end_of_phase_1) / phase_2["initial_speed"] + phase_1["duration"]
                # 残すは最後の減速区間
                if d <= end_of_phase_3:
                    delta = d - end_of_phase_2
                    v_max = phase_3["initial_speed"]
                    return ((v_max ** 2 + 2 * phase_3["ACC"] * delta)**0.5 - v_max) / phase_3["ACC"] + phase_1["duration"] + phase_2["duration"]
                # ここは間に合ってない人たち
                delta = d - end_of_phase_3
                v_exit = phase_3["initial_speed"] + phase_3["duration"] * phase_3["ACC"]
                return delta / v_exit + phase_1["duration"] + phase_2["duration"] + phase_3["duration"]

        if len(speed_profile) == 2:
            # ADの場合のみ
            phase_1 = speed_profile[0]
            phase_2 = speed_profile[1]
            v_0 = phase_1["initial_speed"]
            end_of_phase_1 = phase_1["ACC"] * phase_1["duration"]**2 * 0.5 + phase_1["duration"] * v_0
            v_max = phase_2["initial_speed"]
            end_of_phase_2 = phase_2["ACC"] * phase_2["duration"]**2 * 0.5 + phase_2["duration"] * v_max + end_of_phase_1
            if d <= end_of_phase_1:
                return ((v_0 ** 2 + 2 * phase_1["ACC"] * d)**0.5 - v_0) / phase_1["ACC"]
            if d <= end_of_phase_2:
                delta = d - end_of_phase_1
                return ((v_max ** 2 + 2 * phase_2["ACC"] * delta)**0.5 - v_max) / phase_2["ACC"] + phase_1["duration"]
            delta = d - end_of_phase_2
            v_exit = phase_2["initial_speed"] + phase_2["duration"] * phase_2["ACC"]
            return delta / v_exit + phase_1["duration"] + phase_2["duration"]

    def convert_profile_to_eta(self, speed_profile, waypoints):
        entrance_time = waypoints[0]["eta"]
        calibrated_ETA_list = []
        v = speed_profile[0]["initial_speed"]
        for phase in speed_profile:
            v += phase["duration"] * phase["ACC"]
        v_exit = v

        arrival_at_orifice_exit = 0
        for idx, waypoint_with_eta in enumerate(waypoints):
            if idx <= self.global_params.ORIFITH_EXIT_INDEX:
                xcor = waypoint_with_eta["x"]
                arrival_time = self.calc_arrival_time(speed_profile, xcor)
                calibrated_ETA = {**waypoint_with_eta, "eta": arrival_time + entrance_time}
                arrival_at_orifice_exit = arrival_time + entrance_time
            else:
                if idx == self.global_params.ORIFITH_EXIT_INDEX + 1:
                    print("出口到達時刻: ", arrival_at_orifice_exit)
                distance_from_previous_exit = waypoint_with_eta["x"] - waypoints[int(self.global_params.ORIFITH_EXIT_INDEX)]["x"]
                calibrated_ETA = {**waypoint_with_eta, "eta": arrival_at_orifice_exit + distance_from_previous_exit / v_exit}
            calibrated_ETA_list.append(calibrated_ETA)
        # print()
        return calibrated_ETA_list

    def calc_ideal_params_at_end(self, waypoints_with_eta):
        gp = self.global_params
        # グループIDとグループの中で何番目の車かを把握
        group_idx = waypoints_with_eta[0]["group_id"]
        order_in_group = waypoints_with_eta[0]["order_in_group"]

        # group_idx, グループ内順位をもとに終点の理想時刻を計算
        ideal_time_for_idx_zero = gp.START_TIME + self.ORIFITH_LENGTH / gp.DESIRED_SPEED + group_idx * gp.WINDOW_SIZE
        ideal_arrive_time_at_end = ideal_time_for_idx_zero + order_in_group * gp.DESIRED_TTC
        # 前の車の到着時刻にTTCを足す
        df = self.waypoint_table
        if order_in_group > 0:
            previous_car_arrival_time = df[(df["group_id"] == group_idx) & (df["order_in_group"] == int(order_in_group - 1)) &
                                           (df["waypoint_idx"] == gp.ORIFITH_EXIT_INDEX)]["eta"].iloc[0]
            ideal_arrive_time_at_end = max(ideal_arrive_time_at_end, previous_car_arrival_time + gp.DESIRED_TTC)
        # print("理想到着時刻", ideal_arrive_time_at_end)

        return {"ideal_arrive_time": ideal_arrive_time_at_end, "ideal_speed": gp.DESIRED_SPEED}

    def plot(self):
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
        car_idx_list = self.waypoint_table["car_idx"].unique()
        waypoints = self.waypoint_table["x"].unique()

        plt.figure(figsize=(6, 6))
        for car_idx in car_idx_list:
            _df = self.waypoint_table
            df_by_car = _df[_df["car_idx"] == car_idx]
            plt.plot(df_by_car["x"], df_by_car["eta"], color=color_list[car_idx % 6], linewidth=1, linestyle='--')
            plt.scatter(df_by_car["x"], df_by_car["eta"], color=color_list[car_idx % 6], alpha=0.7, s=20)

        plt.axvspan(xmin=self.ORIFITH_LENGTH, xmax=max(waypoints), facecolor='lightgray', alpha=0.4)

        x_ticks = waypoints
        plt.xticks(x_ticks)
        # 罫線を引く
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('ETA')
