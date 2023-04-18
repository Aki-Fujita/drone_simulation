from functions.opt_velocity import optimal_velocity
import numpy as np
import matplotlib.pyplot as plt


class Drones:
    def __init__(self, xcor, ycor, v_0, a, c, legal_speed, scale_factor, drone_idx, min_acc=0, x_end=1e6) -> None:
        self.xcor = xcor
        self.ycor = ycor
        self.v_x = v_0
        self.a = a
        self.c = c
        self.max_speed = legal_speed
        self.min_acc = min_acc  # ここでは減速度として使っている。正数が入ってくることに注意
        self.x_end = x_end
        self.scale_factor = scale_factor
        self.idx = drone_idx
        self.stop_distance = legal_speed**2 / min_acc / 2 if self.min_acc != 0 else None
        self.target_height = self.stop_distance  #一旦シミュレーションの動画ようにy座標を与える
        self.xcorList = [xcor]
        self.v_xList = [v_0]
        self.ycorList = [v_0]
        self.isFinished = False

    def update(self, delta_t, delta_x):
        # self.v_x = min(optimal_velocity(self.c, self.a, delta_x, delta_t, self.v_x), self.max_speed)
        self.v_x = optimal_velocity(self.c, self.a, delta_x, delta_t, self.v_x)
        self.xcor += self.v_x * delta_t
        self.ycor = min(self.ycor + self.v_x * delta_t, self.target_height)

        # ycorの制御
        distance_to_goal = self.x_end - self.xcor
        if distance_to_goal < self.stop_distance:
            self.ycor = (distance_to_goal / self.stop_distance) * self.target_height

    def leader_update(self, delta_t):
        # リーダー（先頭）の場合はゴールとの距離に応じて減速モードにする
        distance_to_goal = self.x_end - self.xcor

        if (self.isFinished):
            pass
        
        # ゴールに到着した判定とその後の処理
        if abs(distance_to_goal) < (0.5 / self.scale_factor) or distance_to_goal < 0:
            self.xcor = 1e6  # 無限大の値を入れて存在しないことにする
            self.ycor = 0  # 無限大の値を入れて存在しないことにする
            self.v_x = 0
            self.isFinished = True

        # 減速モードの判定
        elif distance_to_goal < self.stop_distance:
            deceleration = min(self.min_acc, (self.v_x ** 2 / (2 * distance_to_goal)))
            # self.v_x = min(self.max_speed, optimal_velocity(self.c, self.a, distance_to_goal, delta_t, self.v_x))
            self.v_x -= deceleration * delta_t
            self.ycor = (distance_to_goal / self.stop_distance) * self.target_height
        else:
            self.v_x += (self.max_speed - self.v_x) * self.a * delta_t
            self.ycor = min(self.ycor + self.v_x * delta_t, self.target_height)
        self.xcor += self.v_x * delta_t
    
    def record(self):
        self.xcorList.append(self.xcor)
        self.v_xList.append(self.v_x)
        self.ycorList.append(self.ycor)

    def force_velocity_change(self, vel_after):
        self.v_x = vel_after

    def plot_history(self, time_step= 1):
        steps = len(self.xcorList)
        t = np.arange(steps) * time_step
        plt.figure(figsize=(5, 10))
        plt.subplot(4, 1, 1)
        plt.plot(t, self.xcorList, color="red", linewidth=0.5)
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("time", fontsize=8)
        plt.ylabel("Distance", fontsize=8)

        plt.subplot(4, 1, 2)
        plt.plot(t, self.v_xList, color="red", linewidth=0.5)
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("time", fontsize=8)
        plt.ylabel("velocity", fontsize=8)

        plt.subplot(4, 1, 3)
        plt.plot(self.xcorList, self.v_xList, color="red", linewidth=0.5)
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("x", fontsize=8)
        plt.ylabel("v", fontsize=8)
        
        t_acc = np.arange(steps-1) * time_step
        plt.subplot(4, 1, 4)
        plt.plot(t_acc, np.diff(self.v_xList), color="red", linewidth=0.5)
        plt.yticks(fontsize=8)
        plt.xticks(fontsize=8)
        plt.xlabel("x", fontsize=8)
        plt.ylabel("acc", fontsize=8)
        
