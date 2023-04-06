from .drones import Drones


class SimulationSettings:
    def __init__(self, drone_num, TOTAL_TIME, time_step, scale_factor):
        self.drone_num = drone_num
        self.TOTAL_TIME = TOTAL_TIME
        self.time_step = time_step
        self.simulation_steps = int(TOTAL_TIME / time_step)
        self.scale_factor = scale_factor
        self.is_initialized = False
        self.result = {}

    def initialize(self):
        self.is_initialized = True

    def run(self):
        if not self.is_initialized:
            raise "initializeメソッドが未実行です"

    def test(self, drone_num):
        if (drone_num == 1):
            leader = Drones(xcor=0, ycor=0, v_0=0, a=1, c=2, legal_speed=2)
            for i in range(self.simulation_steps):
                leader.leader_update(self.time_step)
                leader.record()

        else:
            print("testはドローン1台でお願いします。")
