from utils import check_multiple_noise_effect, validate_with_ttc
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tqdm.notebook import tnrange
from .BaseSimulationModel import BaseSimulation


class DFRSimulation(BaseSimulation):
    def __init__(self,  **kwargs):
        super().__init__(**kwargs)  # 親クラスのコンストラクタを呼び出す
        # 他モデルのインポート
        self.CARS = kwargs.get("CARS")
        self.waypoints = kwargs.get("waypoints")
        self.reservation_table = kwargs.get("reservation_table")
        # ノイズ関係の諸元
        self.NOISE_PROBABILITY = kwargs.get("NOISE_PROBABILITY")
        # だいたい何秒先のノイズならわかるか、に相当する数字.
        self.FUTURE_SCOPE = kwargs.get("FUTURE_SCOPE")
        # だいたい何秒先のノイズならわかるか、に相当する数字.
        self.MEAN_NOISE_PERIOD = kwargs.get("MEAN_NOISE_PERIOD")
        self.create_noise = kwargs.get(
            "create_noise", self.create_noise_default)
        self.state = {}

    def conduct_simulation(self, should_plot=False):
        current_noise = []
        cars_on_road = []
        next_car_idx = 0
        previous_state = {
            "noiseList": [],
            "influenced_by_noise_cars": [],
            "influenced_by_eta_cars": []
        }
        for i in tnrange(self.total_steps, desc="Simulation Progress"):
            next_car = self.CARS[next_car_idx]
            time = i * self.TIME_STEP
            event_flg = None
            self.state = {
                "time": time,
                "next_car": next_car_idx,
            }

            """
            STEP 0. 到着する車がいれば到着
            """
            if time >= next_car.arrival_time:
                cars_on_road.append(next_car)
                next_car_idx = cars_on_road[-1].car_idx + 1
                event_flg = "arrival"
            cars_on_road = [
                car for car in cars_on_road if car.xcor < self.TOTAL_LENGTH]  # ゴールしたものは除く

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

            """
            STEP 2. ノイズの影響を受ける車と、ノイズによって影響を受けた他の車の影響を受けた車をリスト化
            """
            influenced_by_noise_cars = []
            if event_flg:
                print()
                print(f"t={time}, next_car={next_car_idx}, current_noise= {
                      current_noise}, event_flg={event_flg}")
                # 新しいノイズが来るか新しい車が到着したら誰が該当するかの判定をする.
                influenced_by_noise_cars = self.find_noise_influenced_cars(
                    cars_on_road, current_noise, time)
                for car in cars_on_road:
                    # noiseを通るETAを計算する（これはノイズに引っ掛かろうがそうでなかろうが全員必須。）
                    car.add_noise_eta(current_noise)
                    self.reservation_table.update_with_request(
                        car_idx=car.car_idx, new_eta=car.my_etas)

            influenced_by_eta_cars = self.find_ETA_influenced_cars(
                cars_on_road)
            influenced_cars = list(
                set(influenced_by_noise_cars + influenced_by_eta_cars))

            if event_flg or len(influenced_cars) > 0:
                print(f"直接ノイズの影響を受けるもの: {influenced_by_noise_cars}")
                print(f"他の車の影響: {influenced_by_eta_cars}")
                print(f"対象車: {influenced_cars}")
                event_flg = "influenced car exists"

            if len(influenced_cars) > 0:  # ETA変更する車が存在した場合.
                """
                STEP 3. 続いて影響されるうち先頭の車のETAをupdateする.
                (a) ノイズ由来での進路変更の場合 = > ノイズだけを気にすればよい.
                (b) 他の車由来での進路変更 = > 前の車だけを気にすればよい.
                (c) 両方の影響を受けた時 = > 前の車だけを気にすればよい.
                """
                car_to_action_id = min(influenced_cars)
                car_to_action = self.CARS[car_to_action_id]
                # 先頭車がノイズの影響だけを受けている場合
                if not car_to_action_id in influenced_by_eta_cars:
                    print(f"t={time}, car_id:{
                          car_to_action_id} avoiding noise.")
                    if car_to_action_id == 0:
                        leader = None
                    else:
                        leader = self.CARS[car_to_action_id-1]
                    new_eta = car_to_action.modify_eta(
                        noiseList=current_noise, table=self.reservation_table, current_time=time, leader=leader)
                else:
                    print(f"t={time}, car_id:{
                          car_to_action_id} changed by leading car.")
                    new_eta = car_to_action.modify_eta(
                        noiseList=current_noise, table=self.reservation_table, current_time=time, leader=self.CARS[car_to_action_id-1])
                self.reservation_table.update_with_request(
                    car_idx=car_to_action_id, new_eta=new_eta)
                car_to_action.my_etas = new_eta
                # print(f"new_eta:\n{new_eta}")

            """
            STEP 4. 全員前進.
            """
            for car in cars_on_road:
                car.decide_speed(time, self.TIME_STEP)
                car.proceed(self.TIME_STEP, time)

            self.record(time, event_flg)
            if should_plot and (i % 5 == 0 or event_flg):
                self.plot_history_by_time(current_noise, time)
            if time % 10 == 0:
                for car in cars_on_road:
                    print(f"t={time}, car_id:{car.car_idx}, xcor:{
                          car.xcor}, speed:{car.v_x}")

    def find_noise_influenced_cars(self, cars_on_road, noiseList, time):
        car_list = [car.car_idx for idx, car in enumerate(
            cars_on_road) if check_multiple_noise_effect(noiseList, car, time)]
        return car_list

    def find_ETA_influenced_cars(self, cars_on_road):
        eta_reservation_table = self.reservation_table.eta_table
        TTC = self.reservation_table.global_params.DESIRED_TTC
        car_list = [car.car_idx for car_id, car in enumerate(cars_on_road) if not validate_with_ttc(
            eta_reservation_table, car.my_etas, TTC)]
        return car_list

    def create_noise_default(self, current_time):
        if current_time % 8 == 0 and current_time > 0:
            # ノイズを未来に発生させる（この瞬間、だと偶然ハマる車が出てきしまう）
            return {"x": [400, 430], "t": [current_time+5, current_time + 8]}
        # この場合はノイズを発生させない.
        return None

    # FIXME: 後々抽象化したい.
    def record(self, time, event_flg):
        """
        平均速度, その時の密度を記録する
        """
        logObj = {
            "time": time,
            "v_mean": 0,
            "density": None,
            "event_flg": None
        }
        cars_on_road = [
            car for car in self.CARS if car.xcor < self.TOTAL_LENGTH]
        logObj["density"] = len(cars_on_road) / self.TOTAL_LENGTH

        if event_flg == "noise created":
            logObj["event_flg"] = "noise"

        if len(cars_on_road) == 0:
            logObj["v_mean"] = 0
            self.v_mean_log.append(logObj)
            return
        v_mean = sum([car.v_x for car in cars_on_road]) / len(cars_on_road)
        logObj["v_mean"] = v_mean
        self.v_mean_log.append(logObj)
        return

    def plot_history_by_time(self, noise_list, current_time):
        """
        各車のETAの変更履歴、座標、ノイズの有無をプロットする。
        """
        color_list = ["orange", "pink", "blue", "brown", "red", "green"]
        car_idx_list_on_road = [
            car.car_idx for car in self.CARS if car.arrival_time <= current_time]
        eta_table = self.reservation_table.eta_table

        plt.figure(figsize=(6, 6))
        ax = plt.gca()

        # 全車のETAのプロット
        car_idx_list = [car.car_idx for car in self.CARS]
        _df = eta_table
        for car_idx in car_idx_list:
            df_by_car = _df[(_df["car_idx"] == car_idx) &
                            (_df["type"] == "waypoint")]
            plt.plot(df_by_car["x"], df_by_car["eta"],
                     color=color_list[car_idx % 6], linewidth=1, linestyle='--', alpha=0.1)
            plt.scatter(df_by_car["x"], df_by_car["eta"],
                        color=color_list[car_idx % 6], alpha=0.2, s=20)

        # 現在道路に入った車に対するプロット
        for car_idx in car_idx_list_on_road:
            df_by_car = _df[(_df["car_idx"] == car_idx) &
                            (_df["type"] == "waypoint")]
            plt.plot(df_by_car["x"], df_by_car["eta"],
                     color=color_list[car_idx % 6], linewidth=1, linestyle='--', alpha=0.1)
            plt.scatter(df_by_car["x"], df_by_car["eta"],
                        color=color_list[car_idx % 6], alpha=0.2, s=20)
            car = self.CARS[car_idx]
            plt.plot(car.xcorList, car.timeLog,
                     color=color_list[car_idx % 6], linewidth=1)
            plt.scatter([car.xcor], [current_time],
                        color=color_list[car_idx % 6], s=40, zorder=5)

        # ノイズ領域の描画
        for noise in noise_list:
            x_range = noise["x"]
            t_range = noise["t"]
            width = x_range[1] - x_range[0]
            height = t_range[1] - t_range[0]
            rect = patches.Rectangle(
                (x_range[0], t_range[0]), width, height, color='green', alpha=0.3)
            ax.add_patch(rect)

        plt.title(f"t={current_time:.1f}")

        # x軸の目盛り
        wpts = _df[_df["type"] == "waypoint"]["x"].unique()
        wpts.sort()  # xの値を昇順に並べ替える（必要に応じて）
        plt.xticks(wpts)

        plt.xlim(0, self.TOTAL_LENGTH + 200)  # x軸の範囲を0から1200に設定
        if current_time > 20:
            plt.ylim(current_time-10, 140+current_time-10)   # y軸の範囲を0から140に設定

        else:
            plt.ylim(0, 140)   # y軸の範囲を0から140に設定

        # 罫線を引く
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('ETA')

        # 保存
        plt.savefig(f"images/dfr/dfr_simulation_t={current_time:.1f}.png")
        plt.close()


def test():
    print(f"============TEST START============")
    print("file name: DFRSimulation.py")


if __name__ == "__main__":
    test()
