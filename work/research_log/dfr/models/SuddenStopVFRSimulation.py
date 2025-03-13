from utils import check_multiple_noise_effect, validate_with_ttc, one_by_one_eta_validator
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tqdm.notebook import tnrange
from .BaseSimulationModel import BaseSimulation
import os
from .Cars import Cars
import logging
sys.path.append("../")

logging.basicConfig(level=logging.INFO)  # INFOレベル以上を表示

default_brake_params = {
    "decel": -1,
    "acc":1,
    "coast_time":0, ##定常走行している時間
    "period": 1
}

"""
20250221開発開始
要件としてはこれまでのDFRと基本的に変わらないが、以下の点が異なる
・ノイズが信号ではなく、ある車の急なブレーキになる. 
・そのノイズを受けて後続の車のETA修正が、即時的に行われる（communication_speed=0）
"""
class SuddenStopVFRSimulation(BaseSimulation):
    def __init__(self,  **kwargs):
        super().__init__(**kwargs)  # 親クラスのコンストラクタを呼び出す
        # 他モデルのインポート
        self.CARS = kwargs.get("CARS")
        self.waypoints = kwargs.get("waypoints")
        self.reservation_table = kwargs.get("reservation_table")
        self.noise_params = kwargs.get("noise_params")
        self.brake_params = kwargs.get("brake_params", default_brake_params)

        self.thresh_speed = abs(self.brake_params["decel"]) * self.brake_params["period"]

        self.dfr_reference = kwargs.get("dfr_reference", None)
        self.car_params = kwargs.get("car_params", {})
        self.v_mean_log = []  # 平均速度のログ. 時間と密度も同時に格納する
        if self.dfr_reference is None:
            self.setup_without_reference(kwargs)
        else:
            self.setup_with_reference()

        self.DENSITY = kwargs.get("DENSITY")
        self.TTC = kwargs.get("TTC", 1.5)
        self.plot_condition = kwargs.get("PLOT_CONDITION", [])

    # このモデルのreferenceにはSuddenStopDFRSimulationを指定する.
    def setup_with_reference(self):
        reference = self.dfr_reference
        self.TIME_STEP = reference.TIME_STEP
        self.TOTAL_TIME = reference.TOTAL_TIME
        self.ONE_SEC_STEP = reference.ONE_SEC_STEP
        self.total_steps = reference.total_steps
        self.TOTAL_LENGTH = reference.TOTAL_LENGTH
        self.CARS = self.setup_cars()  # 元の車の情報を使って新たにインスタンスを生成.
        self.observation_points = reference.observation_points
        self.segment_length = reference.segment_length
        self.flow_count_interval = reference.flow_count_interval
        self.exclude_distance = 1.0
        self.cross_counts_record = {x_line: 0 for x_line in self.observation_points}
        self.density_records = {x_line: [] for x_line in self.observation_points}
        self.arrival_time_log = []
        self.noise_params = reference.noise_params
        self.brake_params = reference.brake_params
        self.thresh_speed = abs(self.brake_params["decel"]) * self.brake_params["period"]
        self.create_brake_obj = reference.create_brake_obj


    def setup_without_reference(self, kwargs):
        self.TIME_STEP = kwargs.get("TIME_STEP")
        self.TOTAL_TIME = kwargs.get("TOTAL_TIME")
        self.ONE_SEC_STEP = int(1/self.TIME_STEP)
        self.total_steps = int(self.TOTAL_TIME / self.TIME_STEP)
        self.TOTAL_LENGTH = kwargs.get("TOTAL_LENGTH")
        self.CARS = kwargs.get("CARS")
        self.FUTURE_SCOPE = kwargs.get("FUTURE_SCOPE")
        self.arrival_times = kwargs.get("arrival_times", [])

    def setup_cars(self):
        arrival_times = []
        car_params = self.car_params
        if self.dfr_reference is not None:
            for car in self.dfr_reference.CARS:
                arrival_times.append(car.arrival_time)
            self.arrival_times = arrival_times

        if len(self.arrival_times) == 0:
            raise ValueError("arrival_times is empty")

        CARS = [Cars(arrival_time=time, index=index, **car_params)
                for index, time in enumerate(arrival_times)]
        return CARS

    def conduct_simulation(self, should_plot=False, **kwargs):
        current_noise = []
        cars_on_road = []
        next_car_idx = 0
        self.prev_positions = [car.xcor for car in self.CARS]
        noise_freq = self.noise_params.get("NOISE_FREQUENCY")
        noise_x = self.noise_params.get("NOISE_START_X")
        print(f"noise_freq: {noise_freq}, noise_x: {noise_x}, thresh_speed: {self.thresh_speed}")

        for i in tnrange(self.total_steps, desc="Simulation Progress VFR"):
            next_car = self.CARS[next_car_idx]
            time = i * self.TIME_STEP
            event_flg = None

            """
            STEP 0. 到着する車がいれば到着
            ・cars_on_roadに追加
            ・cars_on_roadの中ですでにゴールした車は除外
            """
            if time >= next_car.arrival_time:
                # 前の車がある程度離れている場合に限り入場許可
                front_car = None if next_car.car_idx == 0 else self.CARS[next_car.car_idx - 1]
                if front_car is None or front_car.xcor >= self.exclude_distance:
                    # next_carの速度を調整する必要あり. 一旦前の車の速度に合わせることにした.  
                    next_car.v_x = front_car.v_x if front_car is not None else next_car.v_x
                    cars_on_road.append(next_car)
                    next_car_idx = cars_on_road[-1].car_idx + 1
                    event_flg = "arrival"
                    self.arrival_time_log.append(time)

            cars_on_road = [
                car for car in cars_on_road if car.xcor < self.TOTAL_LENGTH]

            """
            STEP 1. ノイズが来るかを判定
            """

            if time > 100 and time % noise_freq == 0:
                car_x_list = [car.xcor for car in cars_on_road]
                target_car_list = [car for car in cars_on_road if car.xcor < noise_x and car.v_x > self.thresh_speed ]
                target_car = target_car_list[0]
                brake_obj_list = self.create_brake_obj(self.brake_params, time, target_car)
                target_car.planned_acc_dict = brake_obj_list              
                  

            if len(cars_on_road) < 1:
                continue

            if event_flg is not None:
                logging.debug(f"t={time}, event_flg={
                      event_flg}, noise={current_noise}")

            """
            STEP 3. 普通に走らせる
            ・先頭車が車かノイズかを判定
            ・先頭車がノイズを避けている車だった場合、自分がノイズを避けられるか判定（target_noiseがNoneじゃなくなる）
            ・先頭車に合わせて速度の調節
            """
            for car in cars_on_road:
                front_car = None if car.car_idx == 0 else self.CARS[car.car_idx - 1]
                if front_car is not None and front_car.xcor >= self.TOTAL_LENGTH:
                    front_car = None
                
                if len(car.planned_acc_dict) > 0 :
                    if car.planned_acc_dict[0]["t_start"] <= time and time < car.planned_acc_dict[-1]["t_end"]:
                        car.planned_acc = car.planned_acc_dict[0]["acc"]
                        car.obey_planned_speed(time)
                        # print(f"t={time:.2f}, car_idx: {car.car_idx}, xcor: {car.xcor:.2f}, v_x: {car.v_x:.2f}")
                        continue
                    else:
                        car.planned_acc_dict=[]

                car.decide_speed_helly(front_car, self.TIME_STEP)
            
            for car in cars_on_road:
                front_car = None if car.car_idx == 0 else self.CARS[car.car_idx - 1]
                car.record_headway(time, front_car)
                car.proceed(self.TIME_STEP, time)
                # if car.car_idx == 55 and time > 175:
                #     print(f"t={time}, car_idx: {car.car_idx}, xcor: {car.xcor}, v_x: {car.v_x}, front_car_xcor: {front_car.xcor}")

                if car.xcor >= self.TOTAL_LENGTH:
                    self.goal_time.append(time)

            """
            STEP 4. 各時刻で行なう全体に対する処理
            ・平均速度の保存
            ・プロット
            """
            
            self.record(time, event_flg, noise_x)
            self.record_headway(time)
            self.record_with_observation_points(time)
            if should_plot and (i % 5 == 0):
                plot_start_time = kwargs.get("plot_start", 0)
                plot_finish_time = kwargs.get("plot_finish", 1000)
                self.plot_cars(time, current_noise, plot_start_time, plot_finish_time)



    def plot_cars(self, current_time, noise_list, plot_start_time, plot_finish_time): 
        """
        各車のtrajectoryとノイズをプロットする
        """
        if current_time < plot_start_time or current_time > plot_finish_time:
            return
        color_list = ["orange", "pink", "blue", "brown", "red", "green",
                      "purple", "darkgoldenrod", "cyan", "magenta", "lime", "teal"]
        plt.figure(figsize=(18, 12))
        ax = plt.gca()
        plt.title(f"t={current_time:.1f}")
        car_idx_list_on_road = [
            car.car_idx for car in self.CARS if car.arrival_time <= current_time and car.xcor < self.TOTAL_LENGTH]

        # 車の位置をプロット
        for car_idx in car_idx_list_on_road:
            car_obj = self.CARS[car_idx]
            # plt.plot(car_obj.xcorList, car_obj.timeLog,
            #          color=color_list[car_idx % 6], linewidth=1, label=f'Car_{car_idx}', alpha=0.01)
            plt.scatter([car_obj.xcor], [current_time],
                        color=color_list[car_idx % 12], s=40, zorder=5, label=f'Car_{car_idx}')

        # 罫線を引く
        plt.grid()
        plt.xlabel('x')
        plt.xlim(0, self.TOTAL_LENGTH + 200)
        if current_time > 20:
            plt.ylim(current_time-10, 140+current_time-10)   # y軸の範囲を0から140に設定

        else:
            plt.ylim(0, 140)   # y軸の範囲を0から140に設定
        plt.ylabel('t')

        # 凡例をグラフの外に表示
        legend = plt.legend(bbox_to_anchor=(1.05, 1),
                            loc='upper left', borderaxespad=0., fontsize='small', ncol=2)

        # 保存
        plt.savefig(f"images/vfr/vfr_simulation_t={current_time:.1f}.png")

        # plt.savefig(f"images/vfr/vfr_simulation_t={current_time:.1f}.png", bbox_inches='tight', bbox_extra_artists=[legend])
        plt.close()

def test():
    print(f"============TEST START============")
    print("file name: DFRSimulation.py")


if __name__ == "__main__":
    test()
