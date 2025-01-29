from tqdm.notebook import tnrange
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from .Cars import Cars
from .BaseSimulationModel import BaseSimulation
import logging
logging.basicConfig(level=logging.INFO)  # INFOレベル以上を表示

class NoiseCar:
    """
    ノイズを便宜上車として扱うためのクラス
    """

    def __init__(self, **kwargs):
        self.xcor = kwargs.get("x")
        self.v_x = 0
        self.car_idx = "noise"


class VFRSimulation(BaseSimulation):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)  # 親クラスのコンストラクタを呼び出す
        self.dfr_reference = kwargs.get("dfr_reference", None)
        self.car_params = kwargs.get("car_params", {})
        self.v_mean_log = []  # 平均速度のログ. 時間と密度も同時に格納する
        if self.dfr_reference is None:
            self.setup_without_reference(kwargs)
        else:
            self.setup_with_reference()

    def setup_with_reference(self):
        reference = self.dfr_reference
        self.TIME_STEP = reference.TIME_STEP
        self.TOTAL_TIME = reference.TOTAL_TIME
        self.ONE_SEC_STEP = reference.ONE_SEC_STEP
        self.total_steps = reference.total_steps
        self.TOTAL_LENGTH = reference.TOTAL_LENGTH
        self.FUTURE_SCOPE = reference.FUTURE_SCOPE
        self.create_noise = reference.create_noise  # ノイズの生成関数を揃える
        self.CARS = self.setup_cars()  # 元の車の情報を使って新たにインスタンスを生成.
        self.observation_points = reference.observation_points
        self.segment_length = reference.segment_length
        self.flow_count_interval = reference.flow_count_interval
        self.last_flow_record_time = 0.0
        self.exclude_distance = 1.0
        self.cross_counts_record = {x_line: 0 for x_line in self.observation_points}
        self.density_records = {x_line: [] for x_line in self.observation_points}
        self.arrival_time_log = []

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

    def plot_cars(self, current_time, noise_list, plot_start_time, plot_finish_time):
        """
        各車のtrajectoryとノイズをプロットする
        """
        if current_time < plot_start_time or current_time > plot_finish_time:
            return
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
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
                        color=color_list[car_idx % 6], s=40, zorder=5, label=f'Car_{car_idx}')

        # # ノイズ領域の描画
        # for noise in noise_list:
        #     x_range = noise["x"]
        #     t_range = noise["t"]
        #     width = x_range[1] - x_range[0]
        #     height = t_range[1] - t_range[0]
        #     rect = patches.Rectangle(
        #         (x_range[0], t_range[0]), width, height, color='green', alpha=0.3)
        #     ax.add_patch(rect)
        
        # ノイズ領域を現在時点に描画
        for noise in noise_list:
            noise_color = "red" if noise["t"][0] <= current_time and noise["t"][1] >= current_time else "orange"
            x_range = noise["x"]
            t_range = noise["t"]
            noise_view_range = [current_time, current_time]
            # print(f"noise_view_range: {t_range}")
            width = x_range[1] - x_range[0]
            height = t_range[1] - t_range[0]
            rect = patches.Rectangle(
                (x_range[0], current_time), width, 1, color=noise_color, alpha=0.3)
            ax.add_patch(rect)

        # 罫線を引く
        # plt.grid()
        plt.xlabel('x')
        plt.xlim(0, self.TOTAL_LENGTH + 200)
        if current_time > 20:
            plt.ylim(current_time-10, 140+current_time-10)   # y軸の範囲を0から140に設定

        else:
            plt.ylim(0, 140)   # y軸の範囲を0から140に設定
        plt.ylabel('t')

        # 凡例をグラフの外に表示
        # legend = plt.legend(bbox_to_anchor=(1.05, 1),
        #                     loc='upper left', borderaxespad=0., fontsize='small', ncol=2)

        # 保存
        plt.savefig(f"images/vfr/vfr_simulation_t={current_time:.1f}.png")

        # plt.savefig(f"images/vfr/vfr_simulation_t={current_time:.1f}.png", bbox_inches='tight', bbox_extra_artists=[legend])
        plt.close()

    def can_avoid_noise(self, car, noise):
        """
        車と先頭車の情報を受け取って、その車がノイズを早避けして良いか判断する
        """
        return False

    def conduct_simulation(self, should_plot=False, **kwargs):
        current_noise = []
        cars_on_road = []
        next_car_idx = 0
        self.prev_positions = [car.xcor for car in self.CARS]

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
            1秒に一回 and 現在のノイズがなかったら、
            NOISE_ARRIVAL_PROBに基づいてノイズを発生させる.
            """
            current_noise = [
                # 現在発生しているノイズ、すでに終わったものは入れない.
                noise for noise in current_noise if noise["t"][1] >= time]

            if i % self.ONE_SEC_STEP == 0 and len(current_noise) < 1:
                new_noise = self.create_noise(time)
                if new_noise:
                    current_noise.append(new_noise)
                    event_flg = "noise created"
            # 新規に追加したノイズに対しても一応フィルター
            current_noise = [
                noise for noise in current_noise if noise["t"][1] >= time]
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
                car.is_crossing = False
                noise = current_noise[0] if len(current_noise) > 0 else None
                front_car = None if car.car_idx == 0 else self.CARS[car.car_idx - 1]
                if front_car is not None and front_car.xcor >= self.TOTAL_LENGTH:
                    front_car = None
                if noise is None:
                    car.decide_speed_helly(front_car, self.TIME_STEP)
                    continue
                target_noise = None
                noise_start_x = noise["x"][0]
                noise_end_x = noise["x"][1]
                is_leader = car.car_idx == 0 or (int(car.car_idx - 1)) not in [
                    car.car_idx for car in cars_on_road]
                if car.xcor < noise_start_x:
                    """
                    以下の条件を満たしたらtarget_noiseを設定する
                    ・自分とノイズの間に車が存在しない
                    ・自分とノイズの間に車が存在するが、その車がis_crossing=Trueである
                    => なので要するに自分とnoiseの間に is_crossing = Falseの車がいなければ設定. 
                    """
                    # もしも自分と一つ前の車の間にノイズだったら、target_noiseを設定する
                    cars_between_me_and_noise = [car for frontCar in cars_on_road if frontCar.xcor >
                                                 car.xcor and frontCar.xcor < noise_start_x and frontCar.is_crossing == False]
                    if len(cars_between_me_and_noise) == 0:
                        target_noise = noise
                    if target_noise is not None:
                        """
                        target_noiseが設定されている場合（要するに自分の一つ前がnoiseの時）
                        (a) ノイズが見えていない => 普通にfront_carを見て走る
                        (b) ノイズが見えていて避けられる => 早避けする
                        (c) ノイズが見えていて避けられない => 止まれるようにする
                        """
                        if car.xcor + car.foreseeable_distance < noise_start_x: # (a) 普通にfront_carを見て走る
                            
                            car.decide_speed_helly(front_car, self.TIME_STEP)
                            # car.decide_speed_helly(None, self.TIME_STEP) # ここで前の車が急にいなくなってめっちゃ加速することが起こる. 
                            continue
                        else:
                            # (b)か(c)で場合分け
                            # もしノイズを避けられる場合
                            # そもそもここに入るのは目の前がnoiseの車
                            # if car.car_idx == 18:
                            #     print(f"time:{time}")
                            #     print(car.will_overtake_noise(target_noise, front_car, time))
                            if car.will_overtake_noise(target_noise, front_car, time):
                                
                                # 早避けすることが決まった
                                car.is_crossing = True
                                car.decide_speed_helly(front_car, self.TIME_STEP)
                                continue
                            # (c) 避けられない場合 => この場合は狙った場所で止まれるようにする.
                            else:
                                car.stop_at_target_x(
                                    noise_start_x, time, self.TIME_STEP)

                                continue
                # まさに今ノイズ区間を渡っている場合
                elif car.xcor < noise_end_x:
                    car.is_crossing = True

                car.decide_speed_helly(front_car, self.TIME_STEP)
            
            for car in cars_on_road:
                front_car = None if car.car_idx == 0 else self.CARS[car.car_idx - 1]
                car.record_headway(time, front_car)
                car.proceed(self.TIME_STEP, time)

                if car.xcor >= self.TOTAL_LENGTH:
                    self.goal_time.append(time)

            """
            STEP 4. 各時刻で行なう全体に対する処理
            ・平均速度の保存
            ・プロット
            """
            noise_x = None
            if len(current_noise) > 0:
                noise_x = current_noise[0]["x"][0]
            
            self.record(time, event_flg, noise_x)
            # self.record_headway(time)
            self.record_with_observation_points(time)
            if should_plot and (i % 5 == 0):
                plot_start_time = kwargs.get("plot_start", 0)
                plot_finish_time = kwargs.get("plot_finish", 1000)
                self.plot_cars(time, current_noise, plot_start_time, plot_finish_time)
