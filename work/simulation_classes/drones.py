from functions.opt_velocity import optimal_velocity

class Drones:
    def __init__(self, xcor, ycor, v_0, a, c) -> None:
        self.xcor = xcor
        self.ycor = ycor
        self.v_x = v_0
        self.a = a
        self.c = c
        self.xcorList = [xcor]
        self.v_xList = [v_0]

    def update(self, delta_t, delta_x):
        self.v_x = optimal_velocity(self.c, self.a, delta_x, delta_t, self.v_x)
        self.xcor += self.v_x * delta_t

    def leader_update(self, delta_t):
        self.v_x = self.v_x
        self.xcor += self.v_x * delta_t

    def record(self):
        self.xcorList.append(self.xcor)
        self.v_xList.append(self.v_x)

    def force_velocity_change(self, vel_after):
        self.v_x = vel_after
        

    def bark(self):
        print("hello")
