import copy
from .ReservationTable import ReservationTable
import pandas as pd
from utils import calc_early_avoid_acc, calc_late_avoid, \
    validate_with_ttc, create_itinerary_from_acc, calc_eta_from_acc, crt_itinerary_from_a_optimized
import sys
from functions import helly
import logging
logging.basicConfig(level=logging.INFO)  # INFOレベル以上を表示
sys.path.append("..")

helly_params_default = {
    "max_accel": 3,
    "min_accel": 4,
    "lambda_1": 0.4,
    "lambda_2": 0.6,
    "d": 3,
    "T_des": 1.8,
}


class Cars:
    def __init__(self, **kwargs):
        self.arrival_time = kwargs.get("arrival_time", 0)
        self.car_idx = kwargs.get("index")
        self.v_mean = kwargs.get("v_mean")
        self.v_max = kwargs.get("v_max")
        self.a_max = kwargs.get("a_max")
        self.a_min = kwargs.get("a_min")  # 許容可能な減速度
        self.a_min_lim = kwargs.get("a_min_lim", -5)  # 急ブレーキ用の減速度
        self.xcor = 0
        self.xcorList = [0]
        self.timeLog = [self.arrival_time]
        self.v_x = kwargs.get("v_mean")
        self.headway = 1e3 # 車間距離, 記録用. 
        self.helly_params = kwargs.get("helly_params", {
                                       **helly_params_default, "max_accel": self.a_max, "min_accel": self.a_min, "v_max": self.v_max, "isRss": False})
        self.my_etas = []  # 自分のETA予定表のこと
        self.delta_from_eta = 0  # ETAからのずれ. DAAで必要以上のブレーキやアクセルが必要になった場合にここの値が変動する.
        self.car_length = 3  # 車の長さ (m)
        self.has_reacted_to_noise = False
        self.deviation_log = [] # ETA通りの運行から逸脱した瞬間を記録するリスト

        # 以下はVFRのシミュレーションで利用するprops
        self.foreseeable_distance = kwargs.get(
            "foreseeable_distance", 160)
        self.is_crossing = False  # 今信号を渡っている最中かどうか
        self.acc_itinerary = [
            {"acc": 0, "t_start": self.arrival_time, "v_0": self.v_x, "t_end": 1e7, "x_start": 0}]
        if self.a_max == None or self.v_max == None:
            raise ValueError("入力されていない項目があります。")

    def create_desired_eta(self, way_points):
        def calc_eta(way_points):
            estimated_time_of_arrival = way_points["x"] / \
                self.v_mean + self.arrival_time
            return {**way_points, "eta": estimated_time_of_arrival, "car_idx": self.car_idx, "type": "waypoint"}
        way_points_with_eta = list(map(calc_eta, way_points))
        self.my_etas = way_points_with_eta
        return way_points_with_eta
    
    def create_desired_eta_when_arrived(self, waypoints, eta_reservation_table, TTC):
        def calc_eta(way_point):
            # 基本ETAを計算
            estimated_time_of_arrival = way_point["x"] / self.v_mean + self.arrival_time
            return {**way_point, "eta": estimated_time_of_arrival, "car_idx": self.car_idx, "type": "waypoint"}

        # 各waypointのETAを計算
        waypoints_with_eta = list(map(calc_eta, waypoints))

        return waypoints_with_eta
        
        

    def get_noise_eta_others(self, table):
        """
        Step1. 自分の一つ前の車のETA情報を取得.
        Step2. その車とTTCを空けるようにコースを決める
        """
        print(f"consideration by idx={self.index}")

        df = table.eta_table
        TTC = table.global_params.DESIRED_TTC
        previous_plan = copy.deepcopy(self.my_etas)
        ETAs_for_leading_car = df[df["car_idx"] == int(self.index-1)]
        new_eta = []
        for idx, row in enumerate(ETAs_for_leading_car):
            if row["eta"] + TTC > previous_plan[idx]["eta"]:
                previous_plan[idx]["eta"] = row["eta"] + TTC
            new_eta.append(previous_plan[idx])
        self.my_etas = new_eta
        return new_eta

    def select_noise_for_early_avoid(self, noiseList, current_time):
        """
        ノイズが複数個あった時に一番優先的に避けるべきノイズを返す（なお、早避け前提）.
        """
        required_speeds = []
        for noise in noiseList:
            margin_time = noise["t"][0] - current_time
            if margin_time <= 0:
                required_speeds.append(-1)
                continue
            noise_end = noise["x"][1]
            required_speed = (noise_end - self.xcor) / margin_time
            required_speeds.append(required_speed)

        if len(required_speeds) == 0:
            logging.debug("早避けすべきノイズがない")
            return None, False
        noise_to_avoid = noiseList[required_speeds.index(max(required_speeds))]
        return noise_to_avoid, True

    def select_noise_for_late_avoid(self, noiseList, current_time):
        """
        ノイズが複数個あった時に一番優先的に避けるべきノイズを返す（なお、late_avoid前提）.
        """
        if len(noiseList) == 0:
            return None
        required_speeds = []
        for noise in noiseList:
            margin_time = noise["t"][1] - current_time
            if margin_time < 0:
                raise ValueError("消滅したノイズなはず、何かおかしい")
            noise_start = noise["x"][0]
            required_speed = (noise_start - self.xcor) / \
                (margin_time + 1e-3)  # division by zeroを避けるため
            required_speeds.append(required_speed)
        logging.debug(noiseList, required_speeds)
        noise_to_avoid = noiseList[required_speeds.index(min(required_speeds))]
        return noise_to_avoid

    def modify_eta(self, noiseList, current_time, table, leader=None):
        """
        Step1. 各ノイズに対して加速してやり過ごせないかを検討する（x-t線図で言う左下を目指す）.
        (a) もしもノイズの右端を横切れてかつ、それで他の車にも影響がない場合はそれを新たな経路にする.
        (b) 上記の達成が不可能な場合はおとなしく左上を目指す.
        (c) もし避けるべきノイズがない場合(これは前の車の進路変更だけを気にすれば良い)
        """
        logging.debug(f"avoidance by idx={self.car_idx}, x={self.xcor}, v_0={self.v_x}")
        can_early_avoid = True
        noise_to_avoid, can_early_avoid = self.select_noise_for_early_avoid(
            noiseList, current_time)
        logging.debug(noise_to_avoid, can_early_avoid)

        # (a)の場合: early_avoidをまずは検討.
        if can_early_avoid:
            temp_acc_itinerary = calc_early_avoid_acc(
                noise_to_avoid, current_time, self, table)
            if not temp_acc_itinerary:
                can_early_avoid = False
        if not can_early_avoid:
            noise_to_avoid = self.select_noise_for_late_avoid(
                noiseList, current_time)
            # print("Early avoid 不可, late avoidの探索開始.")
            temp_acc_itinerary = calc_late_avoid(
                noise_to_avoid, current_time, self, table, leader)

        """この時点でtemp_acc_itineraryは早いものか遅いものが何かしら入っている
            ただし、いずれの場合も未認証.
        """

        # print("==== START CREATING ETA====")
        ideal_eta = create_itinerary_from_acc(
            car_obj=self, current_time=current_time, acc_itinerary=temp_acc_itinerary)
        # print(f"ID: {self.car_idx}の新しいETA（Validate前）", ideal_eta)
        # if self.car_idx == 34:
        #     print("L167")
        #     print(ideal_eta)
        #     print()
        #     print(temp_acc_itinerary)
        # 普通に計画すると前の車にぶつかることがあり得る。
        # なのでvalidateが通ったらmy_etasとacc_itineraryを登録
        if validate_with_ttc(table.eta_table, ideal_eta, table.global_params.DESIRED_TTC, car_position=self.xcor, current_time=current_time):
            self.my_etas = ideal_eta
            self.acc_itinerary = temp_acc_itinerary
            return ideal_eta

        # (b)の場合
        print(f"car_idx={self.car_idx}, acc_itinerary={temp_acc_itinerary} \n ideal_eta={ideal_eta}")
        print("Value Error 基本的にここには来ないはず")
        raise ValueError("ノイズを避けることができませんでした。")

    def add_noise_eta(self, noiselist):
        """
        この関数はノイズをETAに新規追加するだけでPUTは行わないことにする.
        """
        current_itinerary = self.my_etas
        noise_x_coors = []
        x_in_itinerary = [point["x"] for point in self.my_etas]
        for noise in noiselist:
            noise_x_coors.append(noise["x"][0])
            noise_x_coors.append(noise["x"][1])

        for noise_x_coor in noise_x_coors:
            """
            noise_x_coorsがすでにETAリストにあったらスキップ
            """
            if noise_x_coor in x_in_itinerary:
                continue
            eta_at_noise = calc_eta_from_acc(noise_x_coor, self.acc_itinerary)
            current_itinerary.append(
                {"eta": eta_at_noise, "car_idx": self.car_idx, "type": "noise", "x": noise_x_coor})

    # DFRのシミュレーション用
    def decide_speed(self, current_time, time_step, front_car=None):
        """
        この関数は自分のacc_itineraryをもとに自分のスピードを決める.
         => ただ、改めて見ると、速度と加速度をただ参照しているだけでは目的地に狙ったETAで到達できるかは怪しい. 
        """
        v_front = front_car.v_x if front_car is not None else None
        
        planned_speed = self.calc_speed_from_acc_itinerary(current_time)
        if planned_speed is None:
            planned_speed = self.v_x
            return
        if v_front is None or self.can_drive_with_planned_speed(front_car, planned_speed):
            self.v_x = planned_speed
            return
        
        # planed_speedからのdeviationが発生する場合
        if not self.can_drive_with_planned_speed(front_car, self.v_x) and current_time > 274.5 and self.car_idx == 74:
            print(f"ID: {self.car_idx}, planned_speed={planned_speed}, v_front={v_front}, v_x={self.v_x}")


        # planned_speedではないにせよ今のスピードで大丈夫な場合
        if self.can_drive_with_planned_speed(front_car, self.v_x):
            return
        
        # 衝突する場合は速度を調整する（最大限のブレーキを踏む）
        controled_speed = max(self.v_x + self.a_min_lim * time_step, 0)
        self.v_x = controled_speed
        self.delta_from_eta += (planned_speed - controled_speed) * time_step # planned_speedに対してどれだけ遅れているかを記録
        logging.debug(f"CONTROL領域: ID: {self.car_idx}, planned_speed={planned_speed}, controled_speed={controled_speed}, v_front={v_front}")

        # 本当はself.delta_from_etaを見た上で余裕がある時には速度を増やしたりしたいが、一旦割愛

    def record_headway(self, time, front_car):
        front_x = front_car.xcor if front_car is not None else 1e6
        self.headway = front_x - self.xcor
        
        

    def can_drive_with_planned_speed(self, front_car, planned_speed):
        """
        前の車との間に十分なスペースがあるかどうかを判断する関数.
        """
        if front_car is None:
            return True
        front_car_brake_distance = abs(front_car.v_x ** 2 / 2 / front_car.a_min)
        my_brake_distance = planned_speed ** 2 / 2 / self.a_min
        headway = front_car.xcor - self.xcor - self.car_length
        return headway + front_car_brake_distance > my_brake_distance

    def calc_speed_from_acc_itinerary(self, time):
        """
        acc_itineraryを元に速度を計算する関数.
        """
        # 区間を検索
        for interval in self.acc_itinerary:
            # 指定された時刻が区間に含まれているか確認
            if interval['t_start'] <= time <= interval['t_end']:
                # 速度を計算: v = v_0 + acc * (time - t_start)
                v_0 = interval['v_0']
                acc = interval['acc']
                t_start = interval['t_start']
                speed = v_0 + acc * (time - t_start)
                return speed
        # 該当する区間がない場合は None を返す
        return None

    def proceed(self, time_step, current_time):
        self.xcor += self.v_x * time_step
        self.xcorList.append(self.xcor)
        self.timeLog.append(current_time)

    # VFRのシミュレーション用に前の車を見ながら速度を決める関数
    def decide_speed_helly(self, front_car, time_step):
        if self.helly_params is None:
            raise ValueError("ヘリーモデル用のパラメータが設定されていません")
        if front_car is None:
            front_car_x = self.xcor + 1000
            front_car_vel = self.v_max
        else:
            front_car_x = front_car.xcor
            front_car_vel = front_car.v_x
        delta_x = front_car_x - self.xcor
        delta_v = front_car_vel - self.v_x
        if delta_x > 700:
            self.v_x = self.v_mean
            return # 自分が先頭の場合や距離が空きすぎている場合は定常走行する.
        v_n = self.v_x

        if delta_x < 0:
            print(f"Error! ID: {self.car_idx}, xcor={self.xcor}, front_car_x={front_car_x}, front_car_id={front_car.car_idx} ")
            print(f"delta_x={delta_x}, delta_v={delta_v}, v_n={v_n}, front_car_vel={front_car_vel}")
            raise ValueError("Overtaking happened")

        next_speed = helly(delta_x, delta_v, time_step, v_n, self.helly_params)
        # if v_n > next_speed:
        #     print(f"ID: {self.car_idx}, 減速. delta_x", delta_x, "delta_v", delta_v, "next_speed", next_speed, "v_n:", v_n,
        #           "front_car_x", front_car_x, "front_car_vel:", front_car_vel)
        self.v_x = next_speed

    def stop_at_target_x(self, target_x, current_time, time_step):
        """
        【停止モードに入っている時の速度と加速度を決める関数】 
        狙った場所に狙った時間より後ろで止まるように加速度を決めた上ですすむ.
        """
        delta_x = target_x - self.xcor
        if self.v_x ** 2 / 2 / self.a_min > delta_x:
            print(f"ID: {self.car_idx}, xcor={self.xcor}, v_x={self.v_x}, delta_x={delta_x}")
            raise ValueError("狙った場所に止まることができません！")

        # 止まれる場合は必要最小限の減速度で減速する.
        a = self.v_x ** 2 / 2 / (delta_x + 1e-2) * -1
        acc = max(a - 0.2, -1 * abs(self.a_min))
        next_speed = self.v_x + acc * time_step
        self.v_x = next_speed
        # print(f"t={current_time}, acc={acc}, next_speed={next_speed}")

    def get_acc_for_time(self, current_time):
        """
        指定された時刻 t に対して、t 以下で最大の t_start を探し、その時の acc を返す。
        :param data: 辞書のリスト。各辞書は acc, t_start, v_0 のキーを持つ。
        :param t: 指定された時刻。
        :return: 条件を満たす acc の値。該当するものがなければ None を返す。
        """
        # t_start が t 以下の要素のみをフィルタリングし、t_start で降順にソート
        valid_items = sorted([item for item in self.acc_itinerary if item['t_start']
                             <= current_time], key=lambda x: x['t_start'], reverse=True)
        # 条件を満たす最初の要素の acc を返す
        return valid_items[0]['acc'] if valid_items else None

    def get_v_for_time(self, current_time):
        valid_items = sorted([item for item in self.acc_itinerary if item['t_start']
                             <= current_time], key=lambda x: x['t_start'], reverse=True)
        return valid_items[0]['v_0']

    # 目の前のノイズを避けるかどうするかを決める関数.
    def will_overtake_noise(self, noise, front_car, time):
        """
        ノイズを渡れるかどうかは2段階に分けて判断する
        (a) そもそもノイズを渡れるかどうか
        →ノイズ発生時刻に、ノイズ終了場所にいないといけない. 
        (b) ノイズを渡った後に前の車にぶつからないかどうか
        """
        noise_start_time = noise["t"][0]
        noise_end_x = noise["x"][1]

        # まずはnoiseを渡れるかどうかを判断
        distance_to_noise = noise_end_x - self.xcor
        # if self.car_idx == 18:
        #     print("L315")
        #     print(distance_to_noise, self.v_x + time >= noise_start_time, )
        if distance_to_noise / (self.v_x+1e-3) + time >= noise_start_time:
            # この場合は等速ではいけない場合なので一旦Falseということにする（挙動の修正次第ではTrueになるかも）
            return False

        """
        ここまで来た時点で等速ならノイズを渡れることが確定している.
        あとは等速で行ったと仮定して前の車にぶつからないかどうかを判断する.
        """
        if front_car is None:
            return True
        
        # if front_car.is_crossing:
        #     logging.debug(f"ID: {self.car_idx}, 前の車が早避け中")
        #     """
        #     前の車が早避け中の場合は自分も渡ると強制的に判断してみる。 
        #     """
        #     return True

        front_car_stoppping_distance = front_car.v_x ** 2 / 2 / front_car.a_min
        distance_between_noiseEnd_and_frontCar = front_car.xcor - noise_end_x
        my_braking_distance = self.v_x ** 2 / 2 / self.a_min

        # if self.car_idx == 18:
        #     print(f"L341の結論: {(front_car.xcor - self.xcor) + front_car_stoppping_distance > my_braking_distance}")
        #     print(f"車間距離: {front_car.xcor - self.xcor}, リーダー停止距離: {front_car_stoppping_distance}, 自分の停止距離:{my_braking_distance}")
        
        if (front_car.xcor - self.xcor) + front_car_stoppping_distance > my_braking_distance + self.v_x * 1.5:
            return True
        
        # 前の車が急ブレーキを踏んだらワンチャン当たる場合=> ガチの急ブレーキでOKならOKにする.  
        if (front_car.xcor - self.xcor) + front_car_stoppping_distance > my_braking_distance*self.a_min / self.a_min_lim + self.v_x * 1.5:
            return True

        return False


def prepare_test():
    return


def test():
    print("============TEST START============")
    current_time = 0.5
    noise = {"t": [10, 13], "x": [220, 240]}
    acc_itinerary_1 = [{"t_start": 0, "acc": 3}, {"t_start": 4, "acc": -1}]
    acc_itinerary_2 = [{"t_start": 4, "acc": 0}]
    carObj = Cars(
        mean_speed=20, acc_itinerary=acc_itinerary_1, a_max=3, max_speed=30)

    sample_table = ReservationTable(waypoints=[],
                                    global_params={"DESIRED_TTC": 3}
                                    )
    carObj.modify_eta(noiseList=[noise],
                      current_time=current_time, table=sample_table)


if __name__ == "__main__":
    test()
