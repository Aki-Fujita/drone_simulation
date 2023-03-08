from functions.opt_velocity import optimal_velocity


class Drones:
    def __init__(self, xcor, ycor, v_0, a, c) -> None:
        self.xcor = xcor
        self.ycor = ycor
        self.v_x = v_0
        self.a = a
        self.c = c

    def update(self, delta_t, delta_x):
        print("update")
        self.v_x = optimal_velocity(self.c, self.a, delta_x, delta_t, self.v_x)
        self.xcor += self.v_x * delta_t
        # print("xcor={0}, v={1}".format(self.xcor, self.v_x))

    def leader_update(self, delta_t):
        self.v_x = self.v_x
        self.xcor += self.v_x * delta_t
        print(self.xcor)
