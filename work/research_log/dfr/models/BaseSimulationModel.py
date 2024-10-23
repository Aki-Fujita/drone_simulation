from utils import check_multiple_noise_effect
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tqdm.notebook import tnrange
from abc import ABC, abstractmethod
import logging
logging.basicConfig(level=logging.INFO)  # INFOレベル以上を表示


class BaseSimulation(ABC):

    def __init__(self, **kwargs):
        self.TOTAL_TIME = kwargs.get("TOTAL_TIME")
        self.TIME_STEP = kwargs.get("TIME_STEP")
        self.ONE_SEC_STEP = int(1/self.TIME_STEP)
        self.total_steps = int(self.TOTAL_TIME / self.TIME_STEP)
        self.TOTAL_LENGTH = kwargs.get("TOTAL_LENGTH")
        self.v_mean_log = []  # 平均速度のログ. 時間と密度も同時に格納する
        self.headway_log = []

    @abstractmethod
    def conduct_simulation(self):
        """
        Conducts the simulation. Must be implemented by subclasses.
        """
        pass

    def record_headway(self, time):
        """
        車間距離と速度のオブジェクトを作る.
        time | headway | v_x | car_idx からなるデータフレームを作成する.
        """
        # シミュレーション中の車をフィルタリング
        cars_on_road = [
            car for car in self.CARS if car.xcor < self.TOTAL_LENGTH and car.arrival_time <= time]

        for i, car in enumerate(cars_on_road):
            car_idx = car.car_idx
            v_x = car.v_x

            if i > 0:
                front_car = cars_on_road[i-1]
                if front_car.xcor >= self.TOTAL_LENGTH:
                    headway = self.TOTAL_LENGTH
                else:
                    headway = front_car.xcor - car.xcor
            else:
                headway = self.TOTAL_LENGTH
            if headway < 0:
                logging.info("time: ", time, "car_idx: ", car_idx, "xcor: ", car.xcor)
                raise ValueError("headway is negative")
            self.headway_log.append({
                "time": time,
                "headway": headway,
                "v_x": v_x,
                "car_idx": car_idx,
                "xcoor": car.xcor,
            })
    
    def record(self, time, event_flg):
        """
        平均速度, その時の密度を記録する
        """
        logObj = {
            "time": time,
            "v_mean": 0,
            "density": None,
            "event_flg": None,
            "flow_sum": 0
        }
        cars_on_road = [
            car for car in self.CARS if car.xcor < self.TOTAL_LENGTH and car.arrival_time <= time]
        logObj["density"] = len(cars_on_road) / self.TOTAL_LENGTH

        if event_flg == "noise created":
            logObj["event_flg"] = "noise"

        if len(cars_on_road) == 0:
            logObj["v_mean"] = 0
            logObj["flow_sum"] = 0
            self.v_mean_log.append(logObj)
            return
        flow_sum = sum([car.v_x for car in cars_on_road])
        v_mean = flow_sum / len(cars_on_road)
        logObj["v_mean"] = v_mean
        logObj["flow_sum"] = flow_sum
        self.v_mean_log.append(logObj)
        return

    def plot_v_mean_log(self, path):
        v_mean_log = self.v_mean_log
        # event_flgが"noise"のデータポイントを抽出する
        noise_time = [entry["time"]
                      for entry in v_mean_log if entry.get("event_flg") == "noise"]
        noise_v_mean = [entry["v_mean"]
                        for entry in v_mean_log if entry.get("event_flg") == "noise"]
        noise_times = [entry["time"]
                       for entry in v_mean_log if entry.get("event_flg") == "noise"]

        # timeとv_meanをそれぞれリストに抽出する
        time = [entry["time"] for entry in v_mean_log]
        v_mean = [entry["v_mean"] for entry in v_mean_log]

        # グラフを描画する
        plt.figure(figsize=(10, 5))
        plt.plot(time, v_mean,)

        # ノイズのデータポイントを赤丸でプロットする
        # plt.scatter(noise_time, noise_v_mean, color='red', label='Noise Event', zorder=5)

        # ノイズ発生タイミングに赤線
        for n in noise_times:
            plt.axvline(x=n, color='orange', linestyle='--', alpha=0.5,
                        linewidth=1, label='Noise Event' if n == noise_times[0] else "")

        # グラフのタイトルとラベルを設定する
        plt.title('Mean Velocity Over Time')
        plt.xlabel('Time')
        plt.ylabel('Mean Velocity')

        plt.xlim(0, self.TOTAL_TIME)
        plt.ylim(0, 35)

        # グリッドを表示する
        plt.grid(True)
        plt.savefig(path)
