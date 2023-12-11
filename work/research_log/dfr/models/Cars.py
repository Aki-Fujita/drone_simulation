class Cars:
    def __init__(self, **kwagrs):
        self.eta_table = []
        self.arrival_time = kwagrs.get("arrival_time")
        self.index = kwagrs.get("index")
        self.mean_speed = kwagrs.get("mean_speed")
        self.xcor = 0
        self.v_x = kwagrs.get("mean_speed")

    def create_desired_list(self, way_points):

        def calc_eta(way_point):
            estimated_time_of_arrival = way_point["x"] / self.mean_speed + self.arrival_time
            return {**way_point, "eta": estimated_time_of_arrival, "car_idx": self.index,
                    "group_id": self.group_id, "order_in_group": self.order_in_grouop}
        way_points_with_eta = list(map(calc_eta, way_points))

        return way_points_with_eta