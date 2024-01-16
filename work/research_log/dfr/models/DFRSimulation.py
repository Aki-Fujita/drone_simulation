from utils import check_multiple_noise_effect, validate_with_ttc
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class DFRSimulation:
    def __init__(self,  **kwargs):
        # シミュレーションのベース諸元
        self.TOTAL_TIME = kwargs.get("TOTAL_TIME")
        self.TIME_STEP = kwargs.get("TIME_STEP")
        self.ONE_SEC_STEP = int(1/self.TIME_STEP)
        self.total_steps = int(self.TOTAL_TIME / self.TIME_STEP)
        self.TOTAL_LENGTH = kwargs.get("TOTAL_LENGTH")

        # 他モデルのインポート
        self.CARS = kwargs.get("CARS")
        self.reservation_table = kwargs.get("reservation_table")

        # ノイズ関係の諸元
        self.NOISE_PROBABILITY = kwargs.get("NOISE_PROBABILITY")
        # だいたい何秒先のノイズならわかるか、に相当する数字.
        self.FUTURE_SCOPE = kwargs.get("FUTURE_SCOPE")
        # だいたい何秒先のノイズならわかるか、に相当する数字.
        self.MEAN_NOISE_PERIOD = kwargs.get("MEAN_NOISE_PERIOD")

    def conduct_simulation(self, should_plot=False):
        current_noise = []
        cars_on_road = []
        next_car_idx = 0
        for i in range(self.total_steps):
            next_car = self.CARS[next_car_idx]
            time = i * self.TIME_STEP
            event_flg = False

            """
            STEP 0. 到着する車がいれば到着
            """
            if time >= next_car.itinerary[0]["eta"]:
                cars_on_road.append(next_car)
                next_car_idx = cars_on_road[-1].index + 1
                event_flg = True

            """
            STEP 1. ノイズが来るかを判定
            1秒に一回 and 現在のノイズがなかったら、
            NOISE_ARRIVAL_PROBに基づいてノイズを発生させる. 
            """
            current_noise = [
                noise for noise in current_noise if noise["t"][1] >= time]  # 現在発生しているノイズ、すでに終わったものは入れない.

            if i % self.ONE_SEC_STEP == 0 and len(current_noise) < 1:
                new_noise = self.create_noise(time)
                current_noise.append(new_noise)

            if len(cars_on_road) < 1:
                continue

            """
            STEP 2. ノイズの影響を受ける車と、ノイズによって影響を受けた他の車の影響を受けた車をリスト化
            """
            influenced_by_noise_cars = self.find_noise_influenced_cars(
                cars_on_road, current_noise)
            influenced_by_eta_cars = self.find_ETA_influenced_cars(
                cars_on_road)
            influenced_cars = list(
                set(influenced_by_noise_cars + influenced_by_eta_cars))

            if event_flg or len(influenced_cars) > 0:
                print()
                print(f"t={time}, next_car={next_car_idx}")
                print(f"current_noise= {current_noise}")
                print(f"直接ノイズの影響を受けるもの: {influenced_by_noise_cars}")
                print(f"他の車の影響: {influenced_by_eta_cars}")
                print(f"対象車: {influenced_cars}")

            if len(influenced_cars) > 0:  # ETA変更する車が存在した場合.
                """
                STEP 3. 続いて影響されるうち先頭の車のETAをupdateする. 
                (a) ノイズ由来での進路変更の場合 => ノイズだけを気にすればよい. 
                (b) 他の車由来での進路変更 => 前の車だけを気にすればよい. 
                (c) 両方の影響を受けた時 => 前の車だけを気にすればよい.
                """
                car_to_action_id = min(influenced_cars)
                car_to_action = self.CARS[car_to_action_id]
                # 先頭車がノイズの影響だけを受けている場合
                if not car_to_action_id in [influenced_by_eta_cars]:
                    new_eta = car_to_action.avoid_noise(
                        noiseList=current_noise, table=self.reservation_table, current_time=time)
                else:
                    new_eta = car_to_action.consider_others(
                        table=self.reservation_table)
                self.reservation_table.update_with_request(
                    car_idx=car_to_action_id, new_eta=new_eta)

            """
            STEP 4. 全員前進. 
            """
            for car in cars_on_road:
                car.decide_speed(time)
                car.proceed(self.TIME_STEP, time)

            if should_plot:
                self.plot_history_by_time(current_noise, time)

    def find_noise_influenced_cars(self, cars_on_road, noiseList):
        car_list = [car.index for idx, car in enumerate(
            cars_on_road) if check_multiple_noise_effect(noiseList, car.itinerary)]
        return car_list

    def find_ETA_influenced_cars(self, cars_on_road):
        eta_reservation_table = self.reservation_table.eta_table
        TTC = self.reservation_table.global_params.DESIRED_TTC
        car_list = [car.index for car_id, car in enumerate(cars_on_road) if not validate_with_ttc(
            eta_reservation_table, car.itinerary, TTC)]
        return car_list

    def create_noise(self, current_time):
        return {"x": [610, 730], "t": [40, 50]}

    def plot_history_by_time(self, noise_list, current_time):
        """
        各車のETAの変更履歴、座標、ノイズの有無をプロットする。
        """
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
        car_idx_list = [car.index for car in self.CARS]
        eta_table = self.reservation_table.eta_table
        waypoints = eta_table["x"].unique()

        plt.figure(figsize=(6, 6))
        ax = plt.gca()

        # 既存のプロットロジック
        for car_idx in car_idx_list:
            _df = eta_table
            df_by_car = _df[_df["car_idx"] == car_idx]
            plt.plot(df_by_car["x"], df_by_car["eta"],
                     color=color_list[car_idx % 6], linewidth=1, linestyle='--')
            plt.scatter(df_by_car["x"], df_by_car["eta"],
                        color=color_list[car_idx % 6], alpha=0.2, s=20)
            car = self.CARS[car_idx]
            plt.plot(car.xcorList, car.timeLog,
                     color=color_list[car_idx % 6], linewidth=1)

        # ノイズ領域の描画
        for noise in noise_list:
            x_range = noise["x"]
            t_range = noise["t"]
            width = x_range[1] - x_range[0]
            height = t_range[1] - t_range[0]
            rect = patches.Rectangle(
                (x_range[0], t_range[0]), width, height, color='green', alpha=0.3)
            ax.add_patch(rect)

        plt.title(f"t={current_time}")

        # x軸の目盛り
        plt.xticks(waypoints)

        # 罫線を引く
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('ETA')

        # 保存
        plt.savefig(f"dfr_simulation_t={current_time}")

        plt.show()
