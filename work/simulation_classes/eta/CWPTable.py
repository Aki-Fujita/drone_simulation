import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass


@dataclass
class GlobalParams:
    WINDOW_SIZE: float
    START_TIME: float
    DESIRED_TTC: float
    DESIRED_SPEED: float
    ORIFITH_LENGTH: int
    V_MAX: float  # 整流区間内で出して良い最大スピード


class CWPTable:
    def __init__(self, **kwargs) -> None:
        self.waypoints = kwargs.get("waypoints")
        self.algorithm = kwargs.get("algorithm", "KISS")
        self.global_params = GlobalParams(**kwargs.get("global_params"))
        self.waypoint_table = pd.DataFrame([])
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

        if self.algorithm == "CONTROLLED":
            print("control")
            enter_speed = kwargs.get("enter_speed")
            max_acc = kwargs.get("max_acc", 0.5)
            max_dec = kwargs.get("max_dec", 0.4)
            car_spec = {"enter_speed": enter_speed, "max_acc": max_acc, "max_dec": max_dec}
            calibrated_list.append(waypoints_with_eta[0])
            ideal_params_at_end = self.calc_ideal_params_at_end(waypoints_with_eta)
            initial_params = {"time": waypoints_with_eta[0]["eta"], "speed": enter_speed}
            eta_controlled = self.calc_controlled_eta(initial_params, ideal_params_at_end, car_spec)

        return calibrated_list

    def calc_ideal_params_at_end(self, waypoints_with_eta):
        gp = self.global_params
        # グループIDとグループの中で何番目の車かを把握
        group_idx = waypoints_with_eta[0]["group_id"]
        order_in_group = waypoints_with_eta[0]["order_in_group"]

        # group_idx, グループ内順位をもとに終点の理想時刻を計算
        orifith_end_index = gp.ORIFITH_LENGTH
        DISTANCE_TO_ORIFITH_END = self.waypoints[orifith_end_index]["x"]
        ideal_time_for_idx_zero = gp.START_TIME + DISTANCE_TO_ORIFITH_END / gp.DESIRED_SPEED + group_idx * gp.WINDOW_SIZE
        ideal_arrive_time_at_end = ideal_time_for_idx_zero + order_in_group * gp.DESIRED_TTC
        print("理想到着時刻", ideal_arrive_time_at_end)

        return {"ideal_arrive_time": ideal_arrive_time_at_end, "ideal_speed": gp.DESIRED_SPEED}

    def calc_controlled_eta(self, initial_params, ideal_params_at_end, car_spec):
        v_0 = initial_params["speed"]
        v_max = self.global_params.V_MAX
        v_exit = ideal_params_at_end["ideal_speed"]
        a_max = car_spec["max_acc"]
        a_dec = car_spec["max_dec"]
        t_0 = initial_params["time"]
        t_end = ideal_params_at_end["ideal_arrive_time"]
        orifith_end_index = self.global_params.ORIFITH_LENGTH
        L_max = self.waypoints[orifith_end_index]["x"]

        if v_0 > v_max or v_exit > v_max:
            ValueError("Enter Speedが最高速度を上回っています")

        # まずは全速力で行けるかを判定
        s_1 = (v_max**2 - v_0 ** 2) / 2 / a_max
        s_3 = (v_max**2 - v_exit ** 2) / 2 / a_dec
        t_1 = (v_max - v_0) / a_max
        t_3 = (v_max - v_exit) / a_dec
        t_2 = t_end - t_0 - t_1 - t_3
        S = s_1 + s_3 + t_2 * v_max
        can_be_realized = True
        if t_2 < 0 or S > L_max:
            can_be_realized = False
        print("達成可能性:", can_be_realized)

        return

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
        x_ticks = waypoints
        plt.xticks(x_ticks)
        # 罫線を引く
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('eta')
