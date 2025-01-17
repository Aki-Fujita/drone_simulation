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
        self.CARS = kwargs.get("CARS")
        self.v_mean_log = []  # 平均速度のログ. 時間と密度も同時に格納する
        self.headway_log = []
        self.goal_time = [] # 各車がゴールした時間を記録する. 

        # 流量計測用の変数
        self.observation_points = kwargs.get("observation_points", [])
        self.segment_length = kwargs.get("segment_length", 500)
        self.flow_count_interval = kwargs.get("flow_count_interval", 10)
        self.flow_results = {} # 断面ベースの流量計測に利用. 
        self.last_flow_record_time = 0.0
        # 観測断面ごとに通過台数をカウントする辞書
        self.cross_counts_record = {x_line: 0 for x_line in self.observation_points}
        self.density_records = {x_line: [] for x_line in self.observation_points}

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
                logging.info(f"time:{time}, car_idx: {car_idx}, xcor:{car.xcor}")
                raise ValueError("headway is negative")
            car.headway = headway
            self.headway_log.append({
                "time": time,
                "headway": headway,
                "v_x": v_x,
                "car_idx": car_idx,
                "xcoor": car.xcor,
            })

    def record_with_observation_points(self, current_time, **kwargs):
        # 初回のみ、記録開始時刻を初期化
        if not hasattr(self, 'last_flow_record_time'):
            self.last_flow_record_time = 0
        # 初回のみ、prev_positionsを初期化
        if not hasattr(self, 'prev_positions'):
            self.prev_positions = [car.xcor for car in self.CARS]
        if not hasattr(self, 'density_records'):
            self.density_records = {x_line: [] for x_line in self.observation_points}

        # 経過時間を計算
        elapsed_time = current_time - self.last_flow_record_time

        # # flow_count_intervalが経過していなければ、密度のみ蓄積
        # for x_line in self.observation_points:
        #     # --- 密度計測範囲（x_line ± segment_length / 2） ---
        #     segment_start = x_line - self.segment_length
        #     segment_end = x_line

        #     # 道路範囲外のチェック
        #     if segment_start < 0:
        #         raise ValueError(f"観測点 {x_line} の密度計測区間が道路の範囲外です。")

        #     # 計測区間に存在する車両をカウント
        #     cars_in_segment = [car for car in self.CARS if segment_start <= car.xcor < segment_end]
        #     density = len(cars_in_segment) / self.segment_length  # 台/m

        #     # 密度を断面ごとに蓄積
        #     self.density_records[x_line].append(density)
        
        # flow_count_intervalが経過していなければ終了
        if elapsed_time < self.flow_count_interval:
            return
        
        # 各観測断面で通過車両数と密度を計測
        # print(f"t={current_time}, 流量計算を実行")
        # print()
        for x_line in self.observation_points:
            # --- 1. 通過車両数のカウント ---
            if x_line not in self.cross_counts_record:
                self.cross_counts_record[x_line] = 0

            for i, car in enumerate(self.CARS):
                prev_pos = self.prev_positions[i]
                curr_pos = car.xcor

                # 観測断面通過の判定
                if prev_pos < x_line <= curr_pos:
                    self.cross_counts_record[x_line] += 1

            segment_start = x_line - self.segment_length
            segment_end = x_line
            cars_in_segment = [car for car in self.CARS if segment_start <= car.xcor < segment_end]
            # print(f"car_list:{[car.car_idx for car in cars_in_segment]}")
            # 流量（台/秒）を計算
            # 愚直に台数をカウントする方法と、速度から計算する方法を二つ行う。
            passed_cars = self.cross_counts_record[x_line]
            flow = passed_cars / elapsed_time  # 台/秒

            avg_speed = sum([car.v_x for car in cars_in_segment]) / len(cars_in_segment) if cars_in_segment else 0.0
            avg_headway = sum([car.headway for car in cars_in_segment]) / len(cars_in_segment) if cars_in_segment else 1e5
            flow_calculated  = avg_speed * (1/(avg_headway + 1e-3))

            # --- 2. 密度の計算 ---

            densities = self.density_records[x_line]
            avg_density = sum(densities) / len(densities) if densities else 0.0
            
            # print(f"x_line={x_line}: {passed_cars}台通過, 流量={flow:.4f} 台/秒, 密度={avg_density:.4f} 台/m, 計算流量={flow_calculated:.4f} 台/秒, 計算密度={(1/avg_headway):.4f} 台/m")

            # --- 3. flow_results に流量と密度を記録 ---
            if x_line not in self.flow_results:
                self.flow_results[x_line] = {'flow': [], 'density': [], 'time': [], "flow_calculated": [], "density_calculated": []}

            self.flow_results[x_line]['flow'].append(flow)
            self.flow_results[x_line]['density'].append(avg_density)
            self.flow_results[x_line]['time'].append(current_time)
            self.flow_results[x_line]['flow_calculated'].append(flow_calculated)
            self.flow_results[x_line]["density_calculated"].append(1/(avg_headway+1e-3))

            # カウントリセット
            self.cross_counts_record[x_line] = 0
            self.density_records[x_line].clear()  # 密度データのリセット

        # 記録時刻を更新
        self.last_flow_record_time = current_time
        self.prev_positions = [car.xcor for car in self.CARS]
        
    
    def record(self, time, event_flg, noise_x=None):
        """
        平均速度, その時の密度を記録する
        """
        logObj = {
            "time": time,
            "v_mean": 0,
            "density": 0,
            "event_flg": 0,
            "flow_sum": 0,
            "flow": 0,
            "car_num_before_noise": 0,
            "car_num_after_noise": 0,
            "v_mean_before_noise": 0,
            "v_mean_after_noise": 0,
            "rho_before_noise": 0,
            "rho_after_noise": 0,
            "flow_before_noise": 0,
            "flow_after_noise": 0,
        }
        cars_on_road = [
            car for car in self.CARS if car.xcor < self.TOTAL_LENGTH and car.arrival_time <= time]
        logObj["density"] = len(cars_on_road) / self.TOTAL_LENGTH * 1000

        if event_flg == "noise created" or event_flg == "noise_continue":
            logObj["event_flg"] = "noise"
        
        # ここからは車がいる前提で話を進める. 
        if len(cars_on_road) == 0:
            logObj["v_mean"] = 0
            logObj["flow_sum"] = 0
            self.v_mean_log.append(logObj)
            return
        flow_sum = sum([car.v_x for car in cars_on_road])
        
        v_mean = flow_sum / len(cars_on_road)
        logObj["v_mean"] = v_mean
        logObj["flow_sum"] = flow_sum
        logObj["flow"] = v_mean * logObj["density"]

        # ここからnoise前後の情報を入れる
        if noise_x is None:
            self.v_mean_log.append(logObj)
            return
        
        car_before_noise = [car for car in cars_on_road if car.xcor <= noise_x]
        car_after_noise = [car for car in cars_on_road if car.xcor > noise_x]
        if len(car_before_noise) == 0:
            self.v_mean_log.append(logObj)
            return
        
        v_mean_before_noise = sum([car.v_x for car in car_before_noise]) / (len(car_before_noise) + 1e-3) 
        v_mean_after_noise = sum([car.v_x for car in car_after_noise]) / (len(car_after_noise)+ 1e-3) 
        rho_before_noise = len(car_before_noise) / noise_x * 1000
        rho_after_noise = len(car_after_noise) / (self.TOTAL_LENGTH - noise_x) * 1000
        logObj["v_mean_before_noise"] = v_mean_before_noise
        logObj["v_mean_after_noise"] = v_mean_after_noise
        logObj["rho_before_noise"] = rho_before_noise
        logObj["rho_after_noise"] = rho_after_noise
        logObj["flow_before_noise"] = v_mean_before_noise * rho_before_noise
        logObj["flow_after_noise"] = v_mean_after_noise * rho_after_noise
        logObj["car_num_before_noise"] = len(car_before_noise)
        logObj["car_num_after_noise"] = len(car_after_noise)

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
