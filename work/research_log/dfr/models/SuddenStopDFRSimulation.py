from utils import check_multiple_noise_effect, validate_with_ttc, one_by_one_eta_validator, print_formatted_dict_list
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tqdm.notebook import tnrange
from .BaseSimulationModel import BaseSimulation
import os
import psutil
import logging
sys.path.append("../")
from utils import find_next_wpt

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
class SuddenStopDFRSimulation(BaseSimulation):
    def __init__(self,  **kwargs):
        super().__init__(**kwargs)  # 親クラスのコンストラクタを呼び出す
        # 他モデルのインポート
        self.CARS = kwargs.get("CARS")
        self.waypoints = kwargs.get("waypoints")
        self.reservation_table = kwargs.get("reservation_table")
        self.noise_params = kwargs.get("noise_params")
        self.brake_params = kwargs.get("brake_params", default_brake_params)
        self.thresh_speed = abs(self.brake_params["decel"]) * self.brake_params["period"]

        # 前の車のETA変更を後ろの車がキャッチするまでの時間
        self.COMMUNICATION_SPEED = kwargs.get(
            "COMMUNICATION_SPEED")  # 前の車のETA変更を後ろの車がキャッチするまでの時間
        
        self.state = {}
        self.DENSITY = kwargs.get("DENSITY")
        self.TTC = kwargs.get("TTC", 1.5)
        self.plot_condition = kwargs.get("PLOT_CONDITION", [])

    def conduct_simulation(self, should_plot=False, **kwargs):
        cars_on_road = []
        next_car_idx = 0
        current_noise = []
        communication_count = 0
        last_eta_updated_car_id = None
        next_car_to_update_eta = None
        did_someone_updated_eta = False
        last_eta_updated_time = 0
        self.prev_positions = [car.xcor for car in self.CARS]
        noise_freq = self.noise_params.get("NOISE_FREQUENCY")
        noise_x = self.noise_params.get("NOISE_START_X")

        waypoints = self.reservation_table.waypoints
        # if eta_reservation_table.index.name != "car_idx":
        #       eta_reservation_table.set_index("car_idx", inplace=True, drop=False)

        for i in tnrange(self.total_steps, desc=f"Simulation Progress DFR, density={self.DENSITY}"):
            next_car = self.CARS[next_car_idx]
            time = i * self.TIME_STEP
            communication_count += self.TIME_STEP
            event_flg = None
            has_sudden_brake_happend = False
            self.state = {
                "time": time,
                "next_car": next_car_idx,
            }

            """
            STEP 0. 到着する車がいれば到着
            """
            if time >= next_car.arrival_time:
                cars_on_road.append(next_car)
                event_flg = "arrival"

                # 続いて予約を取らせる. 
                desired_eta_list = next_car.create_desired_eta_when_arrived(waypoints, self.reservation_table.eta_table, self.TTC)
                is_valid = self.reservation_table.validate(desired_eta_list)
                if is_valid:
                    self.reservation_table.register(desired_eta_list)
                    next_car.my_etas = desired_eta_list

                else:
                    # print(f"car_idx={next_car.car_idx}", "desired eta list:", desired_eta_list)
                    # print( self.reservation_table.eta_table)
                    leader = None if next_car.car_idx < 1 else self.CARS[next_car.car_idx-1]
                    next_car.my_etas = desired_eta_list
                    new_eta = next_car.modify_eta(
                        noiseList=[], table=self.reservation_table, current_time=time, leader=leader)
                    self.reservation_table.update_with_request(
                        car_idx=next_car.car_idx, new_eta=new_eta)
                    next_car.my_etas = new_eta

                next_car_idx = cars_on_road[-1].car_idx + 1

            """
            到着時処理終了
            """

            cars_on_road = [
                car for car in cars_on_road if car.xcor < self.TOTAL_LENGTH]  # ゴールしたものは除く

            """
            STEP 1. ノイズを発生させる（レーンに乱入してくる車を想定する）
            ・まずは指定されたX座標近辺で対象となる車を選択する. 
            ・その車はETAを修正する.
            ・その車のETA変更は即時的に他の車に伝播する. 
            """

            if time > 0 and time % noise_freq == 0:
                target_car_list = [car for car in cars_on_road if car.xcor < noise_x and car.v_x > self.thresh_speed ]
                if len(target_car_list) > 0:
                    target_car = target_car_list[0]
                    # print(f"t={time:.2f}, sudden_brake発生, 対象車: {target_car.car_idx}")
                    brake_obj_list = self.create_brake_obj(self.brake_params, time, target_car)                    
                    new_eta = target_car.sudden_brake(time, brake_obj_list)
                    self.reservation_table.update_with_request(car_idx=target_car.car_idx, new_eta=new_eta)
                    target_car.my_etas = new_eta
                    communication_count = 0
                    has_sudden_brake_happend = True
                else:
                    print(f"t={time:.2f}, 対象車なし")
            
            """
            STEP 2. ノイズの影響を受ける車と、ノイズによって影響を受けた他の車の影響を受けた車をリスト化. 
            ノイズがある場合は、ノイズの影響を受ける車と他の車の影響を受ける車のうち、最も先頭のものから更新. 
            ノイズがない場合はcomminucation_speedごとにETAを更新手続きを行う.
            """
            if len(cars_on_road) < 1:
                continue
            
            # communication_speedごとにETAを更新するためのフラグ. communication_speedが溜まったらETAを更新する車の判定をする
            if has_sudden_brake_happend and communication_count >= self.COMMUNICATION_SPEED:
                influenced_by_noise_cars = []
                influenced_by_eta_cars = self.find_first_ETA_influenced_car(
                    cars_on_road, time)
                influenced_cars = list(
                    set(influenced_by_noise_cars + influenced_by_eta_cars))
            else:
                influenced_cars = []
                influenced_by_noise_cars = []
                influenced_by_eta_cars = []
                

            if event_flg or len(influenced_cars) > 0:
                logging.debug(f"---------t={time:.2f}---------")
                logging.debug(f"対象車: {influenced_cars}")
                # メモリ使用量のチェック
                logging.debug(f"メモリ使用量: {psutil.virtual_memory().percent}%")
                event_flg = "influenced car exists"

            if len(influenced_cars) > 0:  # ETA変更する車が存在した場合.
                # print(f"t={time:.2f}, ETA変更対象車: {influenced_cars}")

                # 通信速度が0の場合は、即時的に全車のETAを更新する.
                """
                注意点: 
                influenced_carsはあくまでもtarget_carのETA変更による影響を受けた車（第一波と呼ぶ）のリストなので、
                第一波がETAを変更した際にそれの影響を受ける車（第二波）, さらに第二波の影響を受ける車... と考慮する必要がある
                """
                if self.COMMUNICATION_SPEED == 0:
                    # for car_to_action_id in influenced_cars:
                    #     car_to_action = self.CARS[car_to_action_id]
                    #     leader = None if car_to_action_id < 1 else self.CARS[car_to_action_id-1]
                    #     new_eta = car_to_action.modify_eta(
                    #     noiseList=current_noise, table=self.reservation_table, current_time=time, leader=leader)
                    #     self.reservation_table.update_with_request(car_idx=car_to_action_id, new_eta=new_eta)
                    #     car_to_action.my_etas = new_eta
                    current_idx = influenced_cars[0]
                    while current_idx < len(self.CARS):
                        car_to_action = self.CARS[current_idx]
                        leader = None if current_idx == 0 else self.CARS[current_idx - 1]
                        new_eta = car_to_action.modify_eta(
                            noiseList=current_noise,
                            table=self.reservation_table,
                            current_time=time,
                            leader=leader
                        )
                        self.reservation_table.update_with_request(car_idx=current_idx, new_eta=new_eta)
                        car_to_action.my_etas = new_eta
                        # print(f"車 {current_idx} のETAを更新")

                        next_idx = current_idx + 1
                        # 次の車が存在する場合、先行車（current_idx の車）のETAとの差が十分でないなら更新対象
                        if next_idx <= max([car.car_idx for car in cars_on_road]):
                            # one_by_one_eta_validator(対象車のETA, 先行車のETA, TTC) が Falseなら影響を受けている
                            if not one_by_one_eta_validator(self.CARS[next_idx].my_etas, car_to_action.my_etas, self.TTC):
                                # 影響を受けるので、次の車も更新対象
                                current_idx = next_idx
                            else:
                                # 次の車のETAが十分な差を持っている場合は、更新ループを終了
                                break
                        else:
                            break


                        
                elif communication_count >= self.COMMUNICATION_SPEED:
                    car_to_action_id = min(influenced_cars)
                    car_to_action = self.CARS[car_to_action_id]
                    leader = None if car_to_action_id < 1 else self.CARS[car_to_action_id-1]

                    new_eta = car_to_action.modify_eta(
                        noiseList=current_noise, table=self.reservation_table, current_time=time, leader=leader)
                    self.reservation_table.update_with_request(
                        car_idx=car_to_action_id, new_eta=new_eta)
                    car_to_action.my_etas = new_eta
                    communication_count = 0
                    last_eta_updated_car_id = car_to_action_id
                    next_car_to_update_eta = car_to_action_id + 1
                    did_someone_updated_eta = True
                    last_eta_updated_time = time
            
            # 誰もETAを変更していない場合、モニタリング機構が発火し、前とあまりに距離が空いている車がいたらETAをアップデートする. ただし、noiseを避けたことがある人に限る. 
            if did_someone_updated_eta and (time - last_eta_updated_time >= self.COMMUNICATION_SPEED) and last_eta_updated_car_id is not None:
                follower = self.CARS[next_car_to_update_eta]
                if follower.has_reacted_to_noise:
                    leader_etas = self.CARS[last_eta_updated_car_id].my_etas
                    follower_x = follower.xcor
                    follower_eta = follower.my_etas

                    # leader_etaとfolower_etaの差が大きすぎた場合にfollowerを更新.
                    follower_next_wp = find_next_wpt(follower_eta, follower_x + 50)
                    leader_next_wp = find_next_wpt(leader_etas, follower_x + 50)

                    if follower_next_wp is not None and leader_next_wp is not None:                        
                        follower_eta = follower_next_wp["eta"]
                        leader_next_eta = leader_next_wp["eta"]

                        if follower_eta - leader_next_eta > 3 * self.TTC:
                            # print("勧告あり")
                            new_eta= follower.modify_eta(noiseList=current_noise, table=self.reservation_table, current_time=time, leader=self.CARS[last_eta_updated_car_id])
                            self.reservation_table.update_with_request(
                                car_idx=follower.car_idx, new_eta=new_eta)
                            # print(new_eta)
                            follower.my_etas = new_eta
                            communication_count = 0
                            last_eta_updated_time = time
                            did_someone_updated_eta = True
                            last_eta_updated_car_id = follower.car_idx
                            next_car_to_update_eta = follower.car_idx + 1
                else:
                    # print("勧告なし")
                    last_eta_updated_car_id = None
                    did_someone_updated_eta = False
                

            """
            STEP 4. 全員前進.
            """
            for car in cars_on_road:
                front_car = None if car.car_idx == 0 else self.CARS[car.car_idx - 1]
                if front_car is not None and front_car.xcor >= self.TOTAL_LENGTH:
                    front_car = None
                car.decide_speed(time, self.TIME_STEP, front_car)
                car.record_headway(time, front_car)
            for car in cars_on_road:
                car.proceed(self.TIME_STEP, time)

                if car.xcor >= self.TOTAL_LENGTH:
                    self.goal_time.append(time)            

            if len(current_noise) > 0:
                noise_x = current_noise[0]["x"][0]
            self.record(time, event_flg, noise_x)
            self.record_headway(time)
            # self.record_with_observation_points(time) # 実際に車の数を数える方法で流量計測
            
            if should_plot and (i % 5 == 0 or event_flg in self.plot_condition):
                plot_start_time = kwargs.get("plot_start", 0)
                plot_finish_time = kwargs.get("plot_finish", 1000)
                self.plot_history_by_time(current_noise, time, plot_start_time, plot_finish_time)
            
            # 基本的にはここには来ないが、ノイズの中に入っていた車がいたらバグなのでシミュレーションを中断する
            if len(current_noise) > 0:
                for car in cars_on_road:
                    xcoor = car.xcor
                    noise_x_range = current_noise[0]["x"]
                    noise_t_range = current_noise[0]["t"]
                    if noise_x_range[0] <= xcoor and xcoor <= noise_x_range[1] and noise_t_range[0] <= time and time <= noise_t_range[1]:
                        print(f"t={time:.2f}, car_idx={car.car_idx}, x={xcoor}")
                        print(f"Error: car {car.car_idx} is in noise area.")
                        raise Exception("Car is in noise area.")

    def find_noise_influenced_cars(self, cars_on_road, noiseList, time, should_print=False):
        car_list = [car.car_idx for idx, car in enumerate(
            cars_on_road) if check_multiple_noise_effect(noiseList, car, time, should_print=should_print)]
        return car_list

    def find_ETA_influenced_cars(self, cars_on_road, time):
        eta_reservation_table = self.reservation_table.eta_table
        TTC = self.reservation_table.global_params.DESIRED_TTC
        car_list = [car.car_idx for car_id, car in enumerate(cars_on_road) if not validate_with_ttc(
            eta_reservation_table, car.my_etas, TTC, car_position=car.xcor, current_time=time)]
        return car_list
    
    """
    上の関数の、一つでも見つかったらすぐ返すバージョン
    """
    def find_first_ETA_influenced_car(self, cars_on_road, time, should_print=False):
        eta_reservation_table = self.reservation_table.eta_table
        TTC = self.reservation_table.global_params.DESIRED_TTC

        for car in cars_on_road:
            if not validate_with_ttc(eta_reservation_table, car.my_etas, TTC, car_position=car.xcor, current_time=time, should_print=should_print):
                return [car.car_idx]  # 条件を満たしたら即リターン

        return []
    
    def create_brake_obj(self, brake_params, time, car):
        brake_obj_list = []
        first_segment = {
            "t_start": time,
            "t_end": time + brake_params["period"],
            "acc": brake_params["decel"],
            "v_0": car.v_x,
            "x_start": car.xcor
        }
        v_after_first_segment = first_segment["v_0"] + first_segment["acc"] * brake_params["period"]
        x_after_first_segment = first_segment["x_start"] + first_segment["v_0"] * brake_params["period"] + 0.5 * first_segment["acc"] * brake_params["period"]**2
        brake_obj_list.append(first_segment)
        if brake_params["coast_time"] > 0:
            second_segment = {
                "t_start": first_segment["t_end"],
                "t_end": first_segment["t_end"] + brake_params["coast_time"],
                "acc": 0,
                "v_0": v_after_first_segment,
                "x_start": x_after_first_segment
            }
            brake_obj_list.append(second_segment)

            x_after_first_segment += v_after_first_segment * brake_params["coast_time"]
        
        # 最後に加速区間を追加
        third_segment = {
            "t_start": first_segment["t_end"] + brake_params["coast_time"],
            "t_end": first_segment["t_end"] + brake_params["coast_time"] + brake_params["period"],
            "acc": brake_params["acc"],
            "v_0": v_after_first_segment,
            "x_start": x_after_first_segment
        }

        brake_obj_list.append(third_segment)
            
        return brake_obj_list

    def plot_history_by_time(self, noise_list, current_time, plot_start_time, plot_finish_time):
        """
        各車のETAの変更履歴、座標、ノイズの有無をプロットする。
        """
        if current_time < plot_start_time or current_time > plot_finish_time:
            return
        save_dir = "images/dfr/"
        # ディレクトリが存在するか確認し、なければ作成
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        color_list = ["orange", "pink", "blue", "brown", "red", "green",
                      "purple", "darkgoldenrod", "cyan", "magenta", "lime", "teal"]
        car_idx_list_on_road = [
            car.car_idx for car in self.CARS if car.arrival_time <= current_time and car.xcor < self.TOTAL_LENGTH]
        eta_table = self.reservation_table.eta_table

        plt.figure(figsize=(18, 12))
        ax = plt.gca()

        # # 全車のETAのプロット これがあるとデバッグしにくいのでコメントアウト.
        # car_idx_list = [car.car_idx for car in self.CARS]
        _df = eta_table
        # for car_idx in car_idx_list:
        #     df_by_car = _df[(_df["car_idx"] == car_idx) &
        #                     (_df["type"] == "waypoint")]
        #     plt.plot(df_by_car["x"], df_by_car["eta"],
        #              color=color_list[car_idx % 12], linewidth=1, linestyle='--', alpha=0.1)
        #     plt.scatter(df_by_car["x"], df_by_car["eta"],
        #                 color=color_list[car_idx % 12], alpha=0.2, s=20)

        # 現在道路に入った車に対するプロット
        for car_idx in car_idx_list_on_road:
            df_by_car = _df[(_df["car_idx"] == car_idx) &
                            (_df["type"] == "waypoint")]
            # ETA線の表示
            plt.plot(df_by_car["x"], df_by_car["eta"],
                     color=color_list[car_idx % 12], linewidth=1, linestyle='--', alpha=0.4)
            plt.scatter(df_by_car["x"], df_by_car["eta"],
                        color=color_list[car_idx % 12], alpha=0.2, s=20)

            car = self.CARS[car_idx]
            # plt.plot(car.xcorList, car.timeLog,
            #          color=color_list[car_idx % 12], linewidth=1, label=f'Car_{car_idx}')
            plt.scatter([car.xcor], [current_time],
                        color=color_list[car_idx % 12], s=40, zorder=5,  label=f'Car_{car_idx}')

        # ノイズ領域の描画
        for noise in noise_list:
            x_range = noise["x"]
            t_range = noise["t"]
            width = x_range[1] - x_range[0]
            height = t_range[1] - t_range[0]
            rect = patches.Rectangle(
                (x_range[0], t_range[0]), width, height, color='green', alpha=0.3)
            ax.add_patch(rect)

        # # 信号としてプロット
        # for noise in noise_list:
        #     noise_color = "red" if noise["t"][0] <= current_time and noise["t"][1] >= current_time else "orange"
        #     x_range = noise["x"]
        #     t_range = noise["t"]
        #     noise_view_range = [current_time, current_time]
        #     # print(f"noise_view_range: {t_range}")
        #     width = x_range[1] - x_range[0]
        #     height = t_range[1] - t_range[0]
        #     rect = patches.Rectangle(
        #         (x_range[0], current_time), width, 2, color=noise_color, alpha=0.8)
        #     ax.add_patch(rect)

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

        # 凡例をグラフの外に表示
        legend = plt.legend(bbox_to_anchor=(1.05, 1),
                            loc='upper left', borderaxespad=0., fontsize='small', ncol=2)

        # 保存処理
        try:
            plt.savefig(f"{save_dir}dfr_simulation_t={current_time:.1f}.png")
            # plt.savefig(
            #     f"{save_dir}dfr_simulation_t={current_time:.1f}.png", bbox_inches='tight', bbox_extra_artists=[legend])
        except Exception as e:
            print(f"Error saving file for time {current_time}: {e}")
        finally:
            plt.close()


def test():
    print(f"============TEST START============")
    print("file name: DFRSimulation.py")


if __name__ == "__main__":
    test()
