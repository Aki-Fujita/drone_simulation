# from .drones import Drones
import numpy as np
import matplotlib.pyplot as plt


class SimulationSettings:
    def __init__(self, TOTAL_TIME, time_step, scale_factor,
                 drone_list= [], boundary_condition='OPEN',
                 TOTAL_DISTANCE = 0):
        self.TOTAL_TIME = TOTAL_TIME
        self.time_step = time_step
        self.simulation_steps = int(TOTAL_TIME / time_step)
        self.scale_factor = scale_factor
        self.drone_list = drone_list
        self.drone_num = len(drone_list)
        self.boundary_condition = boundary_condition
        self.TOTAL_DISTANCE = TOTAL_DISTANCE

        if self.boundary_condition not in ["OPEN", "FIXED"]:
            raise ValueError("boundary_conditionの値が不適切です")

        if (self.boundary_condition == "FIXED" and self.TOTAL_DISTANCE == 0):
            raise ValueError("TOTAL_DISTANCEが未入力です")

    def run(self):
        drone_list = self.drone_list
        if len(self.drone_list) < 1:
            print("ドローンがありません")
            return
        for step in range(self.simulation_steps):
            for idx, drone_i in enumerate(drone_list):
                isLeader = (idx == 0) or \
                    (idx > 0 and drone_list[int(idx-1)].isFinished)
                isFinished = drone_i.isFinished
                
                if isLeader or isFinished:
                    drone_i.leader_update(self.time_step)
                    drone_i.record()
                else:
                    delta_x = drone_list[int(idx-1)].xcor - drone_i.xcor
                    if delta_x < 0:
                        print(f"idx={idx}")
                        print(f"先行車(id={idx})のx座標", drone_list[int(idx - 1)].xcor)
                        print("followerのx座標", drone_list[idx].xcor)
                        raise ValueError("追い抜きが発生しました")
                                      
                    drone_i.update(self.time_step, delta_x)
                    drone_i.record()

    def run_fixed(self):
        drone_list = self.drone_list
        print(drone_list)

    def test(self):
        if (self.boundary_condition == "FIXED"):
            print("固定境界条件")
            self.run_fixed()
          
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
