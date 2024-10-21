import numpy as np

def generate_arrival_times_poisson(lmd, TTC, car_num):
    
    # 指数分布に従って到着時間間隔を生成
    arrival_intervals = np.random.exponential(scale=lmd, size=car_num)

    # 車間時間が min_interval 秒未満の場合は修正
    arrival_intervals = np.maximum(arrival_intervals, TTC)

    # 到着時間の配列（累積和）
    arrival_times = np.cumsum(arrival_intervals)

    return arrival_times.tolist()
