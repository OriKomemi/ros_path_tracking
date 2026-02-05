from typing import Optional
from .math_utils import clamp


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_min: float = -1e9, i_max: float = 1e9):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_min = float(i_min)
        self.i_max = float(i_max)

        self.integral = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 1e-6:
            return 0.0

        self.integral = clamp(self.integral + error * dt, self.i_min, self.i_max)
        d_err = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * d_err
