import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass
import matplotlib.patches as patches


@dataclass
class GlobalParams:
    DESIRED_TTC: float


class ReservationTable:
    def __init__(self, **kwargs) -> None:
        self.waypoints = kwargs.get("waypoints")
        self.eta_table = pd.DataFrame([])
        self.global_params = GlobalParams(**kwargs.get("global_params"))

    def validate(self, waypoints_with_eta):
        result = self.validate_with_ttc(waypoints_with_eta)
        return result

    def validate_with_ttc(self, waypoints_with_eta):
        df = self.eta_table
        if df.shape[0] < 1:
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

    def calibrate_list(self, **kwargs):
        df = self.eta_table
        calibrated_list = []
        if "desired_list" not in kwargs:
            raise ValueError("キーワード引数 'desired_list' が必要です。")

        waypoints_with_eta = kwargs.get("desired_list")

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

    def register(self, waypoints_with_eta):
        df_to_add = pd.DataFrame(waypoints_with_eta)
        combined_df = pd.concat([self.eta_table, df_to_add])
        self.eta_table = combined_df

    def update_with_request(self, **request):
        print("======update occured!=======")
        df = self.eta_table
        car_idx = request.get("car_idx")
        new_eta = request.get("new_eta")
        print(f"car_idx={car_idx}")
        # print(f"new_eta={new_eta}")
        if car_idx == None or new_eta == None:
            raise ValueError(
                "car_idx and new_eta must be specified in the request")
        car_0 = pd.DataFrame(new_eta)
        new_df = df[df['car_idx'] != car_idx]
        # 更新された部分集合を元のDataFrameに追加
        new_df = pd.concat([new_df, car_0], ignore_index=True)
        self.eta_table = new_df

    """
    If noise_list is empty [], it just plots ETAs of each car.
    """

    def plot_with_noise(self, noise_list):
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
        car_idx_list = self.eta_table["car_idx"].unique()
        waypoints = self.eta_table["x"].unique()

        plt.figure(figsize=(6, 6))
        ax = plt.gca()

        # 既存のプロットロジック
        for car_idx in car_idx_list:
            _df = self.eta_table
            df_by_car = _df[(_df["car_idx"] == car_idx)&(_df["type"]=="waypoint")]
            plt.plot(df_by_car["x"], df_by_car["eta"],
                     color=color_list[car_idx % 6], linewidth=1, linestyle='--')
            plt.scatter(df_by_car["x"], df_by_car["eta"],
                        color=color_list[car_idx % 6], alpha=0.3, s=20)

        # ノイズ領域の描画
        for noise in noise_list:
            x_range = noise["x"]
            t_range = noise["t"]
            width = x_range[1] - x_range[0]
            height = t_range[1] - t_range[0]
            rect = patches.Rectangle(
                (x_range[0], t_range[0]), width, height, color='green', alpha=0.3)
            ax.add_patch(rect)

        # x軸の目盛り

        wpts = _df[_df["type"] == "waypoint"]["x"].unique()
        wpts.sort()  # xの値を昇順に並べ替える（必要に応じて）
        plt.xticks(wpts)


        # 罫線を引く
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('ETA')

        plt.show()
