from functions.opt_velocity import optimal_velocity


class Drones:
    def __init__(self, xcor, ycor, v_0, a, c, legal_speed) -> None:
        self.xcor = xcor
        self.ycor = ycor
        self.v_x = v_0
        self.a = a
        self.c = c
        self.max_speed = legal_speed
        self.xcorList = [xcor]
        self.v_xList = [v_0]

    def update(self, delta_t, delta_x):
        # self.v_x = min(optimal_velocity(self.c, self.a, delta_x, delta_t, self.v_x), self.max_speed)
        self.v_x = optimal_velocity(self.c, self.a, delta_x, delta_t, self.v_x)
        self.xcor += self.v_x * delta_t

    def leader_update(self, delta_t):
        self.v_x += (self.max_speed - self.v_x) * self.a * delta_t
        self.xcor += self.v_x * delta_t

    def record(self):
        self.xcorList.append(self.xcor)
        self.v_xList.append(self.v_x)

    def force_velocity_change(self, vel_after):
        self.v_x = vel_after
        
