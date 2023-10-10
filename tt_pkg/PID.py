from tt_pkg.config import config

class PID:
    def __init__(self, kp, ki, kd, err_max, output_max, output_min) -> None:
        self.set_value = 0.0
        self.actual_value = 0.0
        self.actual_value_last = 0.0
        self.err = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.output = 0.0
        self.err_max = err_max
        self.output_max = output_max
        self.output_min = output_min

    def calc(self):
        self.err = self.set_value - self.actual_value
        if self.err ** 2 > self.err_max ** 2:
            if self.err > 0:
                self.err = self.err_max
            else:
                self.err = -self.err_max

        p_out = self.kp * self.err
        i_out = self.ki * self.integral
        d_out = self.kd * (self.actual_value_last - self.actual_value)

        if self.err * i_out < 0:
            i_out = 0.1 * i_out

        self.output = p_out + i_out + d_out

        if self.output ** 2 > self.output_max ** 2:
            if self.output > 0:
                self.output = self.output_max
            else:
                self.output = -self.output_max
        if self.output ** 2 < self.output_min ** 2:
            self.output = 0

        self.actual_value_last = self.actual_value
        self.integral = self.integral + self.err

    def update(self, set_value, actual_value):
        self.set_value = set_value
        self.actual_value = actual_value
        self.calc()
        return float(self.output)

c_kp, c_ki, c_kd = config.get("pid_c")
c_err_max = config.get("c_err_max")
c_max = config.get("c_max")
c_min = config.get("c_min")

v_kp, v_ki, v_kd = config.get("pid_v")
v_err_max = config.get("v_err_max")
v_max = config.get("v_max")
v_min = config.get("v_min")

w_kp, w_ki, w_kd = config.get("pid_w")
w_err_max = config.get("w_err_max")
w_max = config.get("w_max")
w_min = config.get("w_min")

pid_c = PID(c_kp, c_ki, c_kd, c_err_max, c_max, c_min)
pid_v = PID(v_kp, v_ki, v_kd, v_err_max, v_max, v_min)
pid_w = PID(w_kp, w_ki, w_kd, w_err_max, w_max, w_min)