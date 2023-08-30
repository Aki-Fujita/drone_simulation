from functions.helly_velocity import helly
import numpy as np
import matplotlib.pyplot as plt


class Cars:
    def __init__(self, **kwagrs):
        self.arrival_time = kwagrs.get("arrival_time")
        self.index = kwagrs.get("index")
        self.mean_speeed = kwagrs.get("mean_speed")
        self.xcor = 0

    def create_desired_list(self, way_points):
        way_points_with_eta = []
        for idx, way_point in enumerate(way_points):
            estimated_time_of_arrival = way_point["x"] / self.mean_speeed + self.arrival_time
            retObj = {**way_point, "eta": estimated_time_of_arrival, "car_idx": self.index}
            way_points_with_eta.append(retObj)

        return way_points_with_eta
    
    def proceed(self, **kwargs):
        self.xcor += kwargs.get("time_step") * self.mean_speeed

            
        
        
