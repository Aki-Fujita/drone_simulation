from utils import check_multiple_noise_effect, validate_with_ttc
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tqdm.notebook import tnrange
from abc import ABC, abstractmethod


class BaseSimulation(ABC):

    def __init__(self, **kwargs):
        self.TOTAL_TIME = kwargs.get("TOTAL_TIME")
        self.TIME_STEP = kwargs.get("TIME_STEP")
        self.ONE_SEC_STEP = int(1/self.TIME_STEP)
        self.total_steps = int(self.TOTAL_TIME / self.TIME_STEP)
        self.TOTAL_LENGTH = kwargs.get("TOTAL_LENGTH")
        self.v_mean_log = []  # 平均速度のログ. 時間と密度も同時に格納する

    @abstractmethod
    def conduct_simulation(self):
        """
        Conducts the simulation. Must be implemented by subclasses.
        """
        pass

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
