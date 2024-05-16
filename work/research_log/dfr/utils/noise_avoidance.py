import sys
sys.path.append("..")
import random
from models import ReservationTable , Cars
from .conduct_optimization import conduct_fuel_optimization
from .optimizer_for_follower import optimizer_for_follower

"""
noiseを早避けするためのacc_itineraryを計算する関数 

a. (1) 前処理: acc_itineraryでcurrent_time以後に始まるものは削除
   (2) そもそも最大加速で辿り着くか検討. 無理だったらFalse
   (3) 続いて、何時につきたいかをランダムに選択
       a. 最短は前の車の時間 - TTC
       b. 最遅はnoise_start
   (4) それに合わせてacc_itineraryを作成. 

b. (1) 続いて、今から一定加速度で加速してnoiseを避ける場合のpathを計算. 
       ただ、この時の加速度は一定範囲のものを選べても良いかもしれない. 
   (2) 経路のvalidate
"""
def calc_early_avoid_acc(noise, current_time, carObj, table ):
    noise_start_time = noise["t"][0]
    noise_end_poisition = noise["x"][1]

    margin_time_to_noise = noise_start_time - current_time
    max_cover_distance = calc_max_cover_distance(margin_time_to_noise, carObj)
    print(f"delta_t={margin_time_to_noise}, \n 到達可能距離:{max_cover_distance}")
    distance_to_noise_end = noise_end_poisition - carObj.xcor
    if distance_to_noise_end > max_cover_distance["distance"]:
        return False
    
    reservation = table.eta_table
    TTC = table.global_params.DESIRED_TTC
    ETA_of_front_car = reservation[reservation["car_idx"] == carObj.car_idx -1]

    earliest_time = calc_earliest_time(carObj, noise_end_poisition, current_time)
    if len(ETA_of_front_car) > 0:
        earliest_time = ETA_of_front_car[ETA_of_front_car["x"] == noise_end_poisition]["eta"].iloc[0] + TTC
        print(earliest_time)
        print(ETA_of_front_car[ETA_of_front_car["x"] == noise_end_poisition])
    print(f"earliest_time: {earliest_time}, noise_start_time: {noise_start_time}")
    ratio = random.uniform(0, 1) # このパラメタが急ぎ度に相当.
    eta_of_noise_end = ratio * earliest_time + (1-ratio) * noise_start_time
    print(f"eta: {eta_of_noise_end}, 最速:{earliest_time}, late:{noise_start_time}")

    if earliest_time > noise_start_time:
        return False
    
    # 続いてこのETAを満たすacc_itineraryを求める. 
    
    acc_itinerary = solve_acc_itinerary(eta_of_noise_end, carObj, current_time, noise)        

    return acc_itinerary

# 遅く避けるときはノイズを気にする！！
def calc_late_avoid(noise, current_time, carObj, table, leader):
    noise_end_time = noise["t"][1]
    noise_start_poisition = noise["x"][0]
    reservation = table.eta_table
    front_car_etas = reservation[reservation["car_idx"] == carObj.car_idx -1]
    te_by_ttc = front_car_etas[front_car_etas["x"] == noise_start_poisition]["eta"].iloc[0] + table.global_params.DESIRED_TTC
    target_time = max(te_by_ttc, noise_end_time)
    print(f"target_time: {target_time}")

    if te_by_ttc < noise_end_time:
        """
        これは自分がノイズの後ろの1台目の車の場合, なのでノイズを避けることだけ考えれば良い.
        20240415時点では、遅く避ける場合の一台目に限り、燃費の最適化を行う.  
        """
        a_optimized, dt, N = conduct_fuel_optimization(
            x0=carObj.xcor,
            v0=carObj.v_x,
            xe=noise_start_poisition,
            te= target_time - current_time,
            a_max=carObj.a_max,
            a_min = carObj.a_min * -1
        ) # ここで最適化計算を実行
    else:
        """
        これは自分の前に1台目としてnoiseを避けた車がいる時, 
        この場合はその車に衝突しないように経路設計をする必要がある, 
        """
        a_optimized, dt, N = optimizer_for_follower(
            follower=carObj,
            xe=noise_start_poisition,
            noise_pass_time=te_by_ttc,
            a_min = carObj.a_min * -1,
            eta_of_leader = front_car_etas,
            leader = leader,
            current_time=current_time,
            ttc = table.global_params.DESIRED_TTC
        ) # ここで最適化計算を実行
    print("========")
    print(a_optimized)
    print("========")

    acc_itinerary = crt_itinerary_from_a_optimized(a_optimized, dt, carObj, current_time, target_time)

    return acc_itinerary

def crt_itinerary_from_a_optimized(a_optimized, dt, carObj, current_time, target_time):
    acc_itinerary = [accObj for accObj in carObj.acc_itinerary if accObj["t_start"] < current_time]
    previous_speed = acc_itinerary[-1]["v_0"] if len(acc_itinerary)>0 else carObj.v_x
    for idx, a in enumerate(a_optimized):
        acc_itinerary.append({
            "acc": a, 
            "t_start":current_time + idx * dt,
            "v_0": previous_speed
        })
        previous_speed += dt * a

    acc_itinerary.append({
         "acc": 0, 
         "t_start": target_time,
         "v_0": previous_speed
    })

    # 続いて、acc_itineraryに対して、全く同じ加速度が続いたらそれを束ねる処理
    retList = []
    for idx, acc in enumerate(acc_itinerary):
        if idx == 0:
            retList.append(acc)
            continue
        if acc["acc"] == retList[-1]["acc"]:
            continue
        else:
            retList.append(acc)
    return retList


def solve_acc_itinerary(eta_of_noise_end, carObj, current_time, noise):
    """
    今回のケースの場合、ベタ踏みして最大速度でいけば到着できることはわかっている. 
    ・加速度はなるべく小さくしたい
    ・ゴールの速度はなるべく小さくしたい
    という制約でこの問題を解く
    """
    acc_itinerary = [acc for acc in carObj.acc_itinerary if acc["t_start"]<current_time]
    distance_to_noise_end = noise["x"][1] - carObj.xcor
    delta_t = eta_of_noise_end - current_time # この時間枠の中で色々しないといけない. 
    S = distance_to_noise_end
    max_cover_distance = calc_cover_distance(carObj.v_max, carObj, delta_t)
    delta_v = carObj.v_max - carObj.v_x
    delta_v_mean = carObj.v_max - carObj.v_mean
    time_to_v_max = delta_v / carObj.a_max
    time_to_v_mean = delta_v_mean / carObj.a_min # 一旦加速度と同じ減速度にする. 
    coast_time = delta_t - time_to_v_max - time_to_v_mean
    print(f"delta_t:{delta_t}, time_to_v_max:{time_to_v_max}")
    print(f"v_e条件のもと行ける距離:{max_cover_distance}. \nノイズとの距離: {S}, 定常走行時間:{coast_time} ")
 
    # まずは最後にv_meanで終了可能な場合
    if S <= max_cover_distance and coast_time > 0:
        v_m = binary_search_for_v(carObj, S, delta_t)
        print(f"v_m: {v_m}")
        if(v_m != -1):
            
            t_1 = (v_m - carObj.v_x)/carObj.a_max
            t_3 = (v_m - carObj.v_mean)/carObj.a_min
            t_2 = delta_t - t_1 - t_3
            acc_itinerary.append({"t_start":current_time, "acc":carObj.a_max, "v_0":carObj.v_x})
            acc_itinerary.append({"t_start":current_time + t_1, "acc":0, "v_0":v_m})
            acc_itinerary.append({"t_start":current_time+ t_1 + t_2, "acc":-1*(carObj.a_min), "v_0":v_m})
            acc_itinerary.append({"t_start":eta_of_noise_end, "acc":0, "v_0":carObj.v_mean})
            return acc_itinerary
    
    # v_meanの終了は無理だが距離はカバーできる場合（そもそもこの関数の前提条件的にみんなここに入る）
    print("can cover but cannot finish with v_e")
    t_1 = (carObj.v_max- carObj.v_x)/carObj.a_max
    S_dash = S - (carObj.v_max**2- carObj.v_x**2) / 2 / carObj.a_max
    t_rem = delta_t - t_1 #区間2,3の時間和
    t_3 = ((carObj.v_max + t_rem - S_dash)*2/carObj.a_min) ** 0.5
    t_2 = t_rem - t_3
    acc_itinerary.append({"t_start":current_time, "acc":carObj.a_max, "v_0":carObj.v_x})
    acc_itinerary.append({"t_start":current_time + t_1, "acc":0, "v_0":carObj.v_max})
    acc_itinerary.append({"t_start":current_time + t_1 + t_2, "acc":carObj.a_min, "v_0":carObj.v_max})
    acc_itinerary.append({"t_start":eta_of_noise_end, "acc":0, "v_0":carObj.v_max})

    
    return acc_itinerary

def calc_cover_distance(v, carObj, delta_t):
    t_1 = (v - carObj.v_x)/carObj.a_max
    t_2 = (v - carObj.v_mean)/carObj.a_min
    coast_time =  delta_t - t_1 - t_2
    S1 = (v**2 - carObj.v_x ** 2) / carObj.a_max / 2
    S2 = coast_time * v
    S3 =  (v**2 - carObj.v_mean ** 2) / carObj.a_min / 2
    return S1 + S2 + S3

def binary_search_for_v(carObj, S, delta_t, epsilon=1e-3):
    low = carObj.v_x
    high = carObj.v_max
    count = 0
    
    # 2分探索
    while low <= high and count < 100:
        mid = (low + high) / 2
        distance = calc_cover_distance(mid, carObj, delta_t)
        if abs(distance - S) < epsilon:  # epsilonで許容誤差を設定
            print(count, mid, distance, S)
            return mid  # Sに十分近い値が見つかった場合
        elif distance < S:
            low = mid 
        else:
            high = mid 
        count += 1
    
    return -1  # 解が見つからなかった場合


def calc_max_cover_distance(margin_time_to_noise, carObj: Cars):
    # 最大限加速したら最大スピードに達する場合
    print(margin_time_to_noise, carObj.a_max, carObj.v_max)
    if margin_time_to_noise * carObj.a_max + carObj.v_x > carObj.v_max:
        delta_t_to_max_speed = (carObj.v_max - carObj.v_x) / carObj.a_max
        return {"distance": carObj.v_x * delta_t_to_max_speed + 0.5 * carObj.a_max * delta_t_to_max_speed ** 2 +
                carObj.v_max * (margin_time_to_noise - delta_t_to_max_speed), "v_end": carObj.v_max}

    # 最大限加速しても最大スピードに乗らない場合
    return {"distance": carObj.v_x * margin_time_to_noise + 0.5 * carObj.a_max * margin_time_to_noise ** 2,
            "v_end": carObj.v_x + margin_time_to_noise * carObj.a_max}


def calc_earliest_time(carObj, noise_end_poisition, current_time):
    """
    先頭の車が最速でノイズに到着可能な時間を返す.
    この時、一旦アクセルベタ踏みにしておく. 
    """
    distance_to_noise_end = noise_end_poisition - carObj.xcor
    v = carObj.v_x
    distance_to_v_max = (carObj.v_max**2 - v**2)/2/carObj.a_max
    # そもそもトップスピードまでいかない場合
    if distance_to_noise_end < distance_to_v_max:
        return ((v**2 + 2*carObj.a_max*distance_to_noise_end)**0.5 - v) / carObj.a_max + current_time
    
    # トップスピードまでいく場合
    t_to_noise_end = (distance_to_noise_end - distance_to_v_max) / carObj.v_max + (carObj.v_max - v) / carObj.a_max
    return t_to_noise_end + current_time


def prepare_car():
     acc_itinerary = [{"t_start": 0, "acc": 3}, {"t_start": 4, "acc": -1}]
     return Cars(
        v_mean=20, acc_itinerary=acc_itinerary, a_max=2,a_min=2, v_max=30)


def test():
    print("============TEST START============")
    current_time = 1
    # table=[]
    noise = {"t": [14, 16], "x": [300, 340]}
    
    carObj = prepare_car()
    print(carObj.v_x, carObj.xcor)

    # result = calc_early_avoid_acc(noise, current_time, carObj, table)
    # print(f"result:{result}")

    solve_acc_itinerary(noise["t"][1]-2,carObj,current_time,noise)


if __name__ == "__main__":
    test()
