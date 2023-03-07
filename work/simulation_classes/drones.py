class Drones:
    def __init__(self, xcor, ycor) -> None:
        self.xcor = xcor
        self.ycor = ycor

    def proceed(self):
        self.xcor += 1

    def print_cors(self):
        print("xcor={0}, ycor={1}".format(self.xcor, self.ycor))