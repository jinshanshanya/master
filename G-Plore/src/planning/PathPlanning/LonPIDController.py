
class IncrementalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.PIDOutput = 0.0  # PID控制器输出
        self.Velocity = 0.0;  # 系统输出值
        self.LastSystemOutput = 0.0  # 上次系统输出值

        self.Error = 0.0  # 输出值与输入值的偏差
        self.LastError = 0.0
        self.LastLastError = 0.0

    # 设置PID控制器参数
    def SetStepSignal(self, StepSignal):
        self.Error = DesiredInput - self.Velocity
        IncrementValue = self.Kp * (self.Error - self.LastError) \
        + self.Ki * self.Error + self.Kd *(self.Error - 2 * self.LastError + self.LastLastError)
        self.PIDOutput += IncrementValue
        self.LastLastError = self.LastError
        self.LastError = self.Error
        return IncrementValue