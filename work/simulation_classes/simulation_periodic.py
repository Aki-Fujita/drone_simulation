# from .drones import Drones
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os


def find_delta_x(drone_list, idx, total_distance, step):
    if len(drone_list) == 1:
        return 1e6, 1e6
       
    delta_from_idx = drone_list[int(idx-1)].xcor - drone_list[idx].xcor
    delta_v = drone_list[int(idx-1)].v_x - drone_list[idx].v_x
    if delta_from_idx < 0:
        delta_from_idx += total_distance
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
            # print("t=", step)
            for idx, drone_i in enumerate(drone_list):
                delta_x, delta_v = find_delta_x(drone_list, idx, self.TOTAL_DISTANCE, step)
                drone_i.decide_speed(self.time_step, delta_x, delta_v)
            
            for idx, drone_i in enumerate(drone_list):
                delta_x, delta_v = find_delta_x(drone_list, idx, self.TOTAL_DISTANCE, step)
                precedent_car = drone_list[int(idx - 1)]
                if delta_x < (drone_i.v_x - precedent_car.v_x) * self.time_step:
                    print("!!!!!!!!!!!!!")
                    print("追い抜き発生")
                    print(f"step={step}_先行車(id={idx-1})のx座標", drone_list[int(idx - 1)].xcor, precedent_car.xcor)
                    print(f"follower(id={idx})のx座標", drone_list[idx].xcor)
                    print("delta_x", delta_x, " v_x", drone_i.v_x, " leader_v_x", precedent_car.v_x)
                    print("接近距離", (drone_i.v_x - precedent_car.v_x) * self.time_step)
                    print("!!!!!!!!!!!!!")
                    return {"step": step, "follower_idx": idx}
                    raise ValueError("追い抜きが発生しました")
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
        
        result = self.run_parallel(self.drone_list)
        return result


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

    def create_video(self, fileName=""):
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
        drones = self.drone_list
        frames = len(self.drone_list[0].xcorList) - 1
        images = []
        radius = 4

        for i in range(frames):
            if (i % 4 == 1):
                continue
            plt.figure(figsize=(4, 4))

            for (droneNum, drone) in enumerate(drones):
                theta = drone.xcorList[int(i)] / self.TOTAL_DISTANCE * 2 * np.pi
                if (droneNum == 0):
                    plt.scatter(radius * np.cos(theta), radius * np.sin(theta), color=color_list[droneNum % 6], s=6)
                else:
                    plt.scatter(radius * np.cos(theta), radius * np.sin(theta), color=color_list[droneNum % 6], s=6)
            plt.xlim(-1*(radius + 1), radius + 1)
            plt.ylim(-1*(radius + 1), radius + 1)
            plt.yticks(fontsize=8)
            plt.xticks(fontsize=8)
            # plt.xlabel("Distance", fontsize=8)
            # plt.ylabel("Height", fontsize=8)
            plt.savefig(f"tmp/frame_{i}.png")
            plt.close()

            filename = 'tmp/frame_{}.png'.format(i)
            img = cv2.imread(filename)
            images.append(img)
            os.remove(f"tmp/frame_{i}.png")
            if (i % 100 == 0):
                print(f"frame_{i}__Done")

        print("動画作成開始")
        output_file = f'tmp/output_circular_DroneNum={self.drone_num}_RSS.mp4'
        if fileName != "":
            output_file = fileName
        fps = 5
        size = (images[0].shape[1], images[0].shape[0])

        # 動画を保存するためのオブジェクトを生成する
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(output_file, fourcc, fps, size)

        # 100枚の画像を動画に書き込む
        for i in range(len(images)):
            out.write(images[i])

        # 動画を保存するための処理を終了する
        out.release()
        print("END")



    
