# from .drones import Drones
import numpy as np
import matplotlib.pyplot as plt


def find_delta_x(drone_list, idx, total_distance):
    if len(drone_list) == 1:
        return 1e6, 1e6
    
    # 追い抜きがなければこれで良いはず
    delta_from_idx = drone_list[int(idx-1)].xcor - drone_list[idx].xcor
    delta_v = drone_list[int(idx-1)].v_x - drone_list[idx].v_x
    if delta_from_idx < 0:
        delta_from_idx += total_distance
    # 念の為チェック:面倒なのであとでやる
    # current_position = drone_list[idx].xcor
    # distance_list = [drone.xcor - current_position for drone in drone_list]
    return delta_from_idx, delta_v


class SimulationPeriodic:
    def __init__(self, TOTAL_TIME, time_step, scale_factor,
                 TOTAL_DISTANCE, drone_list= [],
                 UPDATE_RULE = "parallel"):
        self.TOTAL_TIME = TOTAL_TIME
        self.TOTAL_DISTANCE = TOTAL_DISTANCE
        self.time_step = time_step
        self.simulation_steps = int(TOTAL_TIME / time_step)
        self.scale_factor = scale_factor
        self.drone_list = drone_list
        self.drone_num = len(drone_list)
        self.UPDATE_RULE = UPDATE_RULE        

    def run_sequential(self, drone_list):
        print("===Sequential実行===")
                
    def run_parallel(self, drone_list):
        print("===PARALLEL実行===")
        for step in range(self.simulation_steps):
            for idx, drone_i in enumerate(drone_list):
                delta_x, delta_v = find_delta_x(drone_list, idx, self.TOTAL_DISTANCE)
                if delta_x < 0:
                    print(f"idx={idx}")
                    print(f"先行車(id={idx})のx座標", drone_list[int(idx - 1)].xcor)
                    print("followerのx座標", drone_list[idx].xcor)
                    raise ValueError("追い抜きが発生しました")
                drone_i.decide_speed(self.time_step, delta_x, delta_v)
            
            for idx, drone_i in enumerate(drone_list):
                drone_i.move(self.time_step, self.TOTAL_DISTANCE)
                drone_i.record()
        print("FINISHED")
        
    def run(self):
        if len(self.drone_list) < 1:
            print("ドローンがありません")
            return
        
        if (self.UPDATE_RULE == "sequential"):
            self.run_sequential(self.drone_list)
            return
        
        self.run_parallel(self.drone_list)

    def test(self):
        if (self.boundary_condition == "FIXED"):
            print("固定境界条件")
          
        else:
            print("testはドローン1台でお願いします。")

    def graph_show(self):
        drone_list = self.drone_list
        t = np.linspace(0, self.TOTAL_TIME, self.simulation_steps + 1)
        plt.figure(figsize=(5, 7))
        plt.subplot(2, 1, 1)
        for drone in drone_list:
            plt.plot(t, np.array(drone.xcorList), color="gray", linewidth=0.5)
        plt.plot(t, np.array(drone_list[0].xcorList), color="red", linewidth=0.5)
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("time", fontsize=8)
        plt.ylabel("xcor", fontsize=8)

        plt.subplot(2, 1, 2)
        for drone in drone_list:
            plt.plot(t, np.array(drone.v_xList), color="gray", linewidth=0.5)
        plt.plot(t, np.array(drone_list[0].v_xList), color="red")
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("time", fontsize=8)
        plt.ylabel("v_x", fontsize=8)

    def graph_show_scaled(self):
        drone_list = self.drone_list
        t = np.linspace(0, self.TOTAL_TIME, self.simulation_steps + 1)
        plt.figure(figsize=(5, 7))
        plt.subplot(2, 1, 1)
        for drone in drone_list:
            plt.plot(t, np.array(drone.xcorList) * self.scale_factor, color="gray", linewidth=0.5)
        plt.plot(t, np.array(drone_list[0].xcorList) * self.scale_factor, color="red", linewidth=0.5)
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.ylim(0, self.TOTAL_DISTANCE*self.scale_factor)
        plt.xlabel("time", fontsize=8)
        plt.ylabel("xcor", fontsize=8)

        plt.subplot(2, 1, 2)
        for drone in drone_list:
            plt.plot(t, np.array(drone.v_xList) * self.scale_factor, color="gray", linewidth=0.5)
        plt.plot(t, np.array(drone_list[0].v_xList) * self.scale_factor, color="red")
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("time", fontsize=8)
        plt.ylabel("v_x", fontsize=8)

    def create_video(self):
        drone_list = self.drone_list
        conducted_steps = len(drone_list[0].xcorList)
        print(conducted_steps)
