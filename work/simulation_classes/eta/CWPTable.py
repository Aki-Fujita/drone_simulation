import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



class CWPTable:
    def __init__(self, **kwargs) -> None:
        self.waypoints = kwargs.get("waypoints")
        self.ttc = kwargs.get("ttc", 3) # time to collision[s]
        self.waypoint_table = pd.DataFrame([])
    
    def validate(self, waypoints_with_eta, algorithm="FIFO"):
        # 初めての車の登録時
        if self.waypoint_table.shape[0] < 1:
            print("初回")
            return True
        validation_list = []
        for idx, waypoint_info in enumerate(waypoints_with_eta):
            if idx == 0:
                validation_list.append(True)
                continue
            
            target_waypoint = waypoint_info["waypoint_idx"]
            filtered_df = self.waypoint_table[self.waypoint_table["waypoint_idx"] == target_waypoint]

            if algorithm == "FIFO":
                # この場合は後から入ったもののetaが一番後であることが条件
                last_entry_time = filtered_df["eta"].max()
                validation_list.append(waypoint_info["eta"] > last_entry_time + self.ttc)
            
            validation_list.append(True)
        return (all(validation_list))
    
    def register(self, waypoints_with_eta):
        df_to_add = pd.DataFrame(waypoints_with_eta)
        combined_df = pd.concat([self.waypoint_table, df_to_add])
        self.waypoint_table = combined_df
    
    def calibrate_list(self, waypoints_with_eta, algorithm="FIFO"):
        df = self.waypoint_table
        calibrated_list = []
        if algorithm == "FIFO":
            for idx, waypoint_info in enumerate(waypoints_with_eta):
                if idx == 0:
                    calibrated_list.append(waypoint_info)
                    continue
                target_waypoint_idx = waypoint_info["waypoint_idx"]
                filtered_df = df[df["waypoint_idx"] == target_waypoint_idx]
                # ここから時間の比較
                last_entry_time = filtered_df["eta"].max()
                desired_entry_time = waypoint_info["eta"]
                if (desired_entry_time < last_entry_time + self.ttc):
                    entry_time = last_entry_time + self.ttc
                else:
                    entry_time = desired_entry_time
                calibrated_list.append({**waypoint_info, "eta": entry_time})

        return calibrated_list
    
    def plot(self):
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
        car_idx_list = self.waypoint_table["car_idx"].unique()

        plt.figure(figsize=(6, 6))
        for car_idx in car_idx_list:
            _df = self.waypoint_table
            df_by_car = _df[_df["car_idx"] == car_idx]
            plt.plot(df_by_car["eta"], df_by_car["x"], color=color_list[car_idx % 6], linewidth=1, linestyle='--') 
            plt.scatter(df_by_car["eta"], df_by_car["x"], color=color_list[car_idx % 6], alpha=0.7, s=20) 