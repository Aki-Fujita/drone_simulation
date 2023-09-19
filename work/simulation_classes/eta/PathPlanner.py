import sys
import numpy as np
import matplotlib.pyplot as plt


sys.path.append("../")


class PathPlanner:
    def __init__(self, **kwargs):
        self.car_spec = kwargs.get("car_spec")
        self.initial_params = kwargs.get("initial_params")
        self.ideal_params_at_end = kwargs.get("ideal_params_at_end")
        self.COURSE_LENGTH = kwargs.get("COURSE_LENGTH")
        self.time_to_exit = self.ideal_params_at_end["ideal_arrive_time"] - self.initial_params["time"]
        self.speed_profile = []

    def decide_is_delayed(self):
        v_0 = self.initial_params["speed"]
        v_lim = self.car_spec["v_max"]
        v_exit = self.ideal_params_at_end["ideal_speed"]

        if v_0 > v_lim or v_exit > v_lim:
            ValueError("Enter Speedが最高速度を上回っています")

        if v_0 > v_exit:
            ValueError("Enter Speedが出口速度を上回っています")

        # 早く着きすぎたか、遅れたか
        is_delayed = self.time_to_exit * v_0 < self.COURSE_LENGTH

        return is_delayed

    def describe_params(self):
        v_0 = self.initial_params["speed"]
        v_lim = self.car_spec["v_max"]
        v_exit = self.ideal_params_at_end["ideal_speed"]
        a_max = self.car_spec["max_acc"]
        a_dec = self.car_spec["max_dec"]
        t_0 = self.initial_params["time"]
        t_end = self.ideal_params_at_end["ideal_arrive_time"]
        params_dict = {"v_0": v_0, "v_lim": v_lim, "v_exit": v_exit,
                       "a_max": a_max, "a_dec": a_dec, "t_0": t_0, "t_end": t_end, "length": self.COURSE_LENGTH}
        return params_dict

    # 限られ時間と距離でそもそもVeまで到達するかを判定
    # {isPossible: Bool,  reason: string}を返す.
    def can_reach_v_exit(self, params):
        delta_v = params["v_exit"] - params["v_0"]
        a_max = params["a_max"]
        time_limit = params["t_end"] - params["t_0"]
        length_limit = params["length"]

        # そもそもスピードに届かない場合
        if delta_v > a_max * time_limit:
            return {"is_possible": False, "Reason": "Time limit"}

        cover_distance = (params["v_exit"]**2 - params["v_0"]**2) / a_max * 0.5

        # 加速区間が足りない場合
        if cover_distance > length_limit:
            return {"is_possible": False, "Reason": "Length limit"}

        # そもそもベタ踏みしてもゴールに届かない場合
        if length_limit > a_max * time_limit**2 * 0.5 + time_limit * params["v_0"]:
            return {"is_possible": False, "Reason": "Cannot Arrive in time"}

        return {"is_possible": True, "Reason": "OK"}

    def calc_distance_by_profile(self, m1, profile, params):

        if profile in ["CAC"]:
            # 加速型の特殊版 (減速ないもの). ただ、カバー範囲はCADと同じになるはず。減速区間がないので変数は加速のタイミングのみ.
            """
            D1: 最初のconstで進む距離（踏み始める時間はm1） 
            D2: 加速距離
            D3: 2回目のconstで進む距離（時間はm3とした） 
            """
            m2 = params["delta_v"] / params["a_max"]
            m3 = params["time_limit"] - m2 - m1  # 総和が time_limitであるという制約
            D1 = params["v_0"] * m1
            D2 = (params["v_exit"] ** 2 - params["v_0"]**2) / params["a_max"] * 0.5  # 途中の加速区間に進む距離
            D3 = (m3 * params["v_exit"])

            # print("m1={0:.2f}, m2={1:.2f}, m3={2:.2f}".format(m1, m2, m3))
            # print("D1={0:.2f}, D2={1:.2f}, D3={2:.2f}".format(D1,D2,D3))

            return D1 + D2 + D3

        # ここから、A,C,Dが全て登場する速度プロファイル
        """
        加速区間 m1[s], 
        等速区間 m2[s],
        減速区間 m3[s]
        のとき必要な距離を計算する関数
        """
        m3 = (m1 * params["a_max"] - params["delta_v"]) / params["a_dec"]  # 最後にv_3になるという制約
        m2 = params["time_limit"] - m3 - m1  # 総和が time_limitであるという制約
        # print()
        # print("calc_distance_by_profile")
        # print("m1={0:.2f}, m2={1:.2f}, m3={2:.2f}".format(m1, m2, m3))

        if profile in ["ACD", "ADC", "CAD"]:
            v_max = params["v_0"] + params["a_max"] * m1
            D1 = (v_max ** 2 - params["v_0"]**2) / params["a_max"] * 0.5  # 加速区間に進む距離
            D3 = (v_max ** 2 - params["v_exit"]**2) / params["a_dec"] * 0.5  # 減速区間で進む距離

            if profile == "ACD":
                D2 = v_max * m2

            if profile == "ADC":
                D2 = params["v_exit"] * m2

            if profile == "CAD":
                D2 = params["v_0"] * m2

            # print("D1={0:.2f}, D2={1:.2f}, D3={2:.2f}".format(D1, D2, D3))

            return D1 + D2 + D3

        if profile in ["CDA", "DAC", "DCA"]:
            v_min = params["v_0"] - params["a_dec"] * m3
            D1 = (params["v_exit"] ** 2 - v_min**2) / params["a_max"] * 0.5  # 加速区間に進む距離
            D3 = (params["v_0"]**2 - v_min**2) / params["a_dec"] * 0.5  # 減速区間で進む距離

            if profile == "CDA":
                D2 = params["v_0"] * m2

            if profile == "DAC":
                D2 = params["v_exit"] * m2

            if profile == "DCA":
                D2 = v_min * m2
            # print("D1={0:.2f}, D2={1:.2f}, D3={2:.2f}".format(D1, D2, D3))
            return D1 + D2 + D3

    def conduct_binary_search(self, **kwargs):
        profile = kwargs.get("profile")
        a = kwargs.get("min_x")
        b = kwargs.get("max_x")
        thresh = kwargs.get("threshold", 0.01)
        params = kwargs.get("params")
        count = 0

        while (b - a) / 2.0 > thresh and count < 100:
            count += 1
            midpoint = (a + b) / 2.0

            # m1に対して単調減少する関数はこっちに入れる
            if profile in ["CAC", "DCA"]:
                if self.calc_distance_by_profile(midpoint, profile, params) > self.COURSE_LENGTH:
                    a = midpoint
                else:
                    b = midpoint
            # 単調増加ならこっち
            else:
                if self.calc_distance_by_profile(midpoint, profile, params) < self.COURSE_LENGTH:
                    a = midpoint
                else:
                    b = midpoint
        # print("二分探索の反復数: ", count)
        return (a + b) / 2.0

    def create_path_by_m1(self, profile, m1, params):
        if profile == "CAC":
            # CACではm1がt_firstとして定義されている。
            t_second = params["delta_v"] / params["a_max"]
            t_third = params["time_limit"] - m1 - t_second
            action_1 = {"ACC": 0, "duration": m1, "initial_speed": params["v_0"]}
            action_2 = {"ACC": params["a_max"], "duration": t_second, "initial_speed": params["v_0"]}
            action_3 = {"ACC": 0, "duration": t_third, "initial_speed": params["v_exit"]}

        if profile == "ACD":
            t_third = (params["a_max"] * m1 - params["delta_v"]) / params["a_dec"]
            t_second = params["time_limit"] - m1 - t_third
            v_max = params["a_max"] * m1 + params["v_0"]
            action_1 = {"ACC": params["a_max"], "duration": m1, "initial_speed": params["v_0"]}
            action_2 = {"ACC": 0, "duration": t_second, "initial_speed": v_max}
            action_3 = {"ACC": params["a_dec"] * (-1), "duration": t_third, "initial_speed": v_max}

        if profile == "DCA":
            t_third = m1
            v_min = params["v_exit"] - m1 * params["a_max"]
            t_first = (params["v_0"] - v_min) / params["a_dec"]
            t_second = params["time_limit"] - t_first - t_third
            action_1 = {"ACC": params["a_dec"] * (-1), "duration": t_first, "initial_speed": params["v_0"]}
            action_2 = {"ACC": 0, "duration": t_second, "initial_speed": v_min}
            action_3 = {"ACC": params["a_max"], "duration": t_third, "initial_speed": v_min}

        return [action_1, action_2, action_3]

    def solve_path(self, priority="speed"):
        """
        下のsolve_path_debugの実験をもとに作成
        """
        params = self.describe_params()
        time_limit = params["t_end"] - params["t_0"]
        delta_v = params["v_exit"] - params["v_0"]
        cover_distance = (params["v_exit"]**2 - params["v_0"]**2) / params["a_max"] * 0.5  # v_0からv_eまでの加速距離
        params["delta_v"] = delta_v
        params["time_limit"] = time_limit

        if priority != "speed":
            ValueError("Priorityの値が不適切です")

        can_reach_v_exit_output = self.can_reach_v_exit(params)
        print(can_reach_v_exit_output)

        if not can_reach_v_exit_output["is_possible"]:
            if can_reach_v_exit_output["Reason"] == "Time limit":
                return [{"ACC": params["a_max"], "duration": time_limit, "initial_speed":params["v_0"]}]
            if can_reach_v_exit_output["Reason"] == "Length limit":
                return [{"ACC": params["a_max"], "duration": time_limit, "initial_speed":params["v_0"]}]

        threshold_distance = cover_distance + (time_limit - delta_v / params["a_max"]) * params["v_0"]
        is_accelerated_profile = self.COURSE_LENGTH >= threshold_distance

        if is_accelerated_profile:
            # 加速型だった場合の解は CAC か ACD のいずれかになるので CAC_MAXとの比較をしてどっちに入るかを判断する
            m1_min_for_acd = delta_v / params["a_max"]
            m1_max_for_acd_by_time = (time_limit - m1_min_for_acd) / (params["a_max"] + params["a_dec"]) * params["a_dec"] + m1_min_for_acd
            m1_max_for_acd_by_speed_limit = (params["v_lim"] - params["v_0"]) / params["a_max"]
            m1_max_for_acd = min(m1_max_for_acd_by_time, m1_max_for_acd_by_speed_limit)
            # ここでACDの最大距離を計算する。もしもACDの最大を超えていたら Impossileの場合の処理をreturn
            max_acd_distance = self.calc_distance_by_profile(m1_max_for_acd, "ACD", params)
            print("ACD_MAX=", max_acd_distance)
            if self.COURSE_LENGTH > max_acd_distance:
                print("ACDでも不可能")
                m3 = (params["v_lim"] - params["v_exit"]) / params["a_dec"]  # 最後にv_3になるという制約
                m2 = params["time_limit"] - m3 - m1_max_for_acd  # 総和が time_limitであるという制約
                if m1_max_for_acd_by_speed_limit < m1_max_for_acd_by_time:
                    return [{"ACC": params["a_max"], "duration": m1_max_for_acd_by_speed_limit, "initial_speed":params["v_0"]},
                            {"ACC": 0, "duration": m2, "initial_speed": params["v_lim"]},
                            {"ACC": params["a_dec"] * (-1), "duration": m3, "initial_speed":params["v_lim"]},]
                else:
                    m2 = time_limit - m1_max_for_acd_by_time
                    return [{"ACC": params["a_max"], "duration": m1_max_for_acd_by_time, "initial_speed":params["v_0"]},
                            {"ACC": params["a_dec"] * (-1), "duration": m2, "initial_speed":params["v_exit"] + m2 * params["a_dec"]}]

            m1_min_for_cac = 0
            m1_max_for_cac = time_limit - delta_v / params["a_max"]
            CAC_MAX = self.calc_distance_by_profile(0, "CAC", params)
            print("CAC_MAX={0:.2f}, course_length={1:.2f}".format(CAC_MAX, self.COURSE_LENGTH))

            profile = "CAC" if self.COURSE_LENGTH <= CAC_MAX else "ACD"
            print("PROFILE: ", profile)
            min_x = m1_min_for_cac if profile == "CAC" else (m1_min_for_acd)
            max_x = m1_max_for_cac if profile == "CAC" else (m1_max_for_acd)
            binary_search_params = {"profile": profile, "params": params, "min_x": min_x,
                                    "max_x": max_x}

            m1 = self.conduct_binary_search(**binary_search_params)
            print("m1の解={0:.2f}".format(m1))
            print("距離:{0:.2f}".format(self.calc_distance_by_profile(m1, profile, params)))

        else:
            # 減速型だった場合の解は DCA 一択
            profile = "DCA"
            print("PROFILE: ", profile)

            m1_min = delta_v / params["a_max"]  # つまりは m3 = 0 の状態

            # 以下 m1_maxの計算。
            max_dec_period_by_v0 = params["v_0"] / params["a_dec"]
            max_dec_period_by_time = (time_limit - m1_min) / (params["a_max"] + params["a_dec"]) * params["a_max"]
            m3_max = min(max_dec_period_by_time, max_dec_period_by_v0)
            m1_max = (m3_max * params["a_dec"] + delta_v) / params["a_max"]

            min_x, max_x = m1_min, m1_max
            binary_search_params = {"profile": profile, "params": params, "min_x": min_x,
                                    "max_x": max_x}
            m1 = self.conduct_binary_search(**binary_search_params)
            # print("m1の解={0:.2f}".format(m1))
            print("距離:{0:.2f}".format(self.calc_distance_by_profile(m1, profile, params)))

        result = self.create_path_by_m1(profile, m1, params)
        self.speed_profile = result
        return result

    def plot_speed_profile(self, partition_num=200):
        if self.speed_profile == []:
            print("speed_profileが未計算です")
            return

        times = np.linspace(0, int(self.time_to_exit), partition_num)
        v_0 = self.initial_params["speed"]

        def calc_speed_by_time(speed_profile, t, v_0):
            thresh_1 = speed_profile[0]["duration"]
            thresh_2 = thresh_1 + speed_profile[1]["duration"]
            v_after_phase_1 = v_0 + speed_profile[0]["ACC"] * speed_profile[0]["duration"]
            v_after_phase_2 = v_after_phase_1 + speed_profile[1]["ACC"] * speed_profile[1]["duration"]
            if t <= thresh_1:
                return v_0 + speed_profile[0]["ACC"] * t
            if t <= thresh_2:
                return v_after_phase_1 + (t - thresh_1) * speed_profile[1]["ACC"]
            return v_after_phase_2 + (t - thresh_2) * speed_profile[2]["ACC"]

        v_xlist = [calc_speed_by_time(self.speed_profile, time, v_0) for time in times]

        # グラフのタイトルや軸ラベルの設定
        plt.plot(times, v_xlist)

        plt.title("Speed Profile")
        plt.xlabel("Time")
        plt.ylabel("v_x")
        plt.grid()
        plt.show()

    def solve_path_debug(self, priority="speed"):
        """
        Path Planning用の関数

        Input: priority = "speed" | "distance"  ※distanceは未実装
        Output: {加速度: xxx, 時間: xxx }[]        
        """
        params = self.describe_params()
        time_limit = params["t_end"] - params["t_0"]
        delta_v = params["v_exit"] - params["v_0"]
        cover_distance = (params["v_exit"]**2 - params["v_0"]**2) / params["a_max"] * 0.5  # v_0からv_eまでの加速距離
        params["delta_v"] = delta_v
        params["time_limit"] = time_limit

        if priority != "speed":
            ValueError("Priorityの値が不適切です")

        # priority = "speed"の場合はまずはそもそもVeで帰れるかを判定する。
        can_reach_v_exit_output = self.can_reach_v_exit(params)
        print(can_reach_v_exit_output)

        if not can_reach_v_exit_output["is_possible"]:
            print("後ほど実装")
            # この場合はとりあえず最大加速するように返しておく

            return []

        # ここから求解を開始する
        """
        基本的に速度のプロファイルは以下の6通り
        (1) ["Acc", "Const", "Dec"] => "ACD"
        (2) ["Acc", "Dec", "Const"] => "ADC"
        (3) ["Const", "Acc", "Dec"] => "CAD"
        (4) ["Const", "Dec", "Acc"] => "CDA"
        (5) ["Dec", "Acc", "Const"] => "DAC",
        (6) ["Dec", "Const", "Acc"] => "DCA"
        以下の中で(1) ~ (3)はCOURSE_LENGTHが一定以上である必要がある. 
        """

        # 二分探索の準備
        count = 0

        threshold_distance = cover_distance + (time_limit - delta_v / params["a_max"]) * params["v_0"]
        # まずは(1) ~ (3)の中に解があるか判定
        print("加速型プロファイルの場合の最低距離:", threshold_distance)
        acc_profile_list = ["CAD", "ADC", "ACD"]
        dec_profile_list = ["CDA", "DAC", "DCA"]

        """
        実験の結果として、
        加速型 => most coverableが"CAD"だが most recommendedが "CAC"または "ACD"なはず。
                 これは、加速型の場合「必要以上に速度を出さないこと」が環境的に大事になるため。
        減速型 => most recommended と most recoverableがいずれも DCA であることがわかった。
        """

        # -----ここの実験の結果、加速型についてはCADが全てを兼ね備えていることがわかったw ------###
        for profile in acc_profile_list:
            m1_min = delta_v / params["a_max"]
            CAD_MIN = self.calc_distance_by_profile(m1_min, profile, params)

            # 続いてCADの場合の最大値を計算
            m1_max = (time_limit - m1_min) / (params["a_max"] + params["a_dec"]) * params["a_dec"] + m1_min
            CAD_MAX = self.calc_distance_by_profile(m1_max, profile, params)
            print("Profile=", profile)
            print("最短の場合: 加速秒数:{m1:.2f} 秒,  最短距離:{distance:.2f} m".format(m1=m1_min, distance=CAD_MIN))
            print("最長の場合: 加速秒数:{m1:.2f} 秒,  最長距離:{distance:.2f} m".format(m1=m1_max, distance=CAD_MAX))
            print()

        # ここからはCACのケース
        m1_min = time_limit - delta_v / params["a_max"]
        CAC_MIN = self.calc_distance_by_profile(m1_min, "CAC", params)
        CAC_MAX = self.calc_distance_by_profile(0, "CAC", params)
        print("CASE: CAC")
        print("最短の場合: 加速秒数:{m1:.2f} 秒,  最短距離:{distance:.2f} m".format(m1=m1_min, distance=CAC_MIN))
        print("最長の場合: 加速秒数:{m1:.2f} 秒,  最長距離:{distance:.2f} m".format(m1=m1_max, distance=CAC_MAX))
        print()

        # -----続いて減速型で実験------#
        for profile in dec_profile_list:

            # まずは減速型の場合の進む距離の最大値を計算
            m1_min = delta_v / params["a_max"]  # つまりは m3 = 0 の状態
            CAD_MIN = self.calc_distance_by_profile(m1_min, profile, params)

            # 続いて減速型の場合の最小値を計算、この場合m3が最大値を取っているはず。m3の最大値はv0による律速か時間による律速
            max_dec_period_by_v0 = params["v_0"] / params["a_dec"]
            max_dec_period_by_time = (time_limit - m1_min) / (params["a_max"] + params["a_dec"]) * params["a_max"]
            m3_max = min(max_dec_period_by_time, max_dec_period_by_v0)
            m1_max = (m3_max * params["a_dec"] + delta_v) / params["a_max"]
            CAD_MAX = self.calc_distance_by_profile(m1_max, profile, params)
            print("Profile=", profile)
            print("最長の場合: 加速秒数:{m1:.2f} 秒, 距離:{distance:.2f} m".format(m1=m1_min, distance=CAD_MIN))
            print("最短の場合: 加速秒数:{m1:.2f} 秒,  距離:{distance:.2f} m".format(m1=m1_max, distance=CAD_MAX))
            print()

        if self.COURSE_LENGTH >= threshold_distance:
            # この場合は(1) ~ (3)の中に解があると考えられる。二分探索の初期値を m1 = (必要加速時間の最低値) とする

            profile = "CAD"
            m1 = delta_v / params["a_max"]

            # まずは小さいところから範囲を絞る
            distance = self.calc_distance_by_profile(m1, profile, params)
            residue = self.COURSE_LENGTH - distance
            print("{count}回目,  距離:{distance:.2f} m".format(count=count, distance=distance))

            if residue > 0.5:  # 残差が大きい場合は次に"CADの際"
                return

        else:
            print("減速型プロファイルになりそう")
