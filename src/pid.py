from time import time

class PID:
    prev_err = 0
    tot_err = 0
    err = None
    output = None
    kp = 0
    ki = 0
    kd = 0
    last_timestamp = None
    maximum = None

    def __init__(self, kp: float, ki: float, kd: float, maximum: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maximum = maximum

    def update_time(self):
        self.last_timestamp = time()

    def update(self, err: float):
        maximum = self.maximum
        time_now = time()
        dtime = time_now - self.last_timestamp
        output = 0
        output += self.kp * err
        self.tot_err += err * dtime
        output += self.tot_err * self.ki
        output += (err - self.prev_err) * self.kd / dtime
        
        self.output = output if abs(output) < maximum else maximum if output > 0 else -maximum
        self.last_timestamp = time_now

    def update_max(self, newmax: float):
        self.maximum = newmax

    def get_output(self):
        return self.output