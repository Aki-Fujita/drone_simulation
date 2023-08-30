import numpy as np
import pandas as pd


class CWPTable:
    def __init__(self, **kwargs) -> None:
        self.waypoints = kwargs.get("waypoints")
        self.waypoint_table = pd.DataFrame([])
    
    def validate(self, waypoints_with_eta, algorithm="FIFO"):
        # 初めての車の登録時
        if self.waypoint_table.shape[0] < 1:
            print("初回")
            return True
        validation_list = []
        for waypoint_info in waypoints_with_eta:
            target_waypoint = waypoint_info["waypoint_idx"]
            filtered_df = self.waypoint_table[self.waypoint_table["waypoint_idx"] == target_waypoint]

            if algorithm == "FIFO":
                # この場合は後から入ったもののetaが一番後であることが条件
                last_entry_time = filtered_df["eta"].max()
                validation_list.append(waypoint_info["eta"] > last_entry_time)
            
            validation_list.append(True)
        return (all(validation_list))
    
    def register(self, waypoints_with_eta):
        df_to_add = pd.DataFrame(waypoints_with_eta)
        combined_df = pd.concat([self.waypoint_table, df_to_add])
        self.waypoint_table = combined_df

        

                

            
        