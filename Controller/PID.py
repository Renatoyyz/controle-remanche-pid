import threading
import time
from Controller.IOs import IO_MODBUS

class PIDController:
    def __init__(self, kp_list = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], ki_list = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], kd_list = [0.05,0.05,0.05,0.05,0.05,0.05], setpoint_list = [180,180,180,180,180,180], io_modbus=None, adr=[1, 2, 3, 4, 5, 6]):
        self.kp_list = kp_list
        self.ki_list = ki_list
        self.kd_list = kd_list
        self.setpoint_list = setpoint_list

        self.value_temp = [0, 0, 0, 0, 0, 0]

        self.io_modbus = io_modbus
        self.adr = adr
        self.integral = [0] * len(adr)
        self.previous_error = [0] * len(adr)
        self._running = False
        self._control_flag = False
        self._thread = None

    def compute(self, current_value, index):
        error = self.setpoint_list[index] - current_value
        self.integral[index] += error
        derivative = error - self.previous_error[index]

        output = self.kp_list[index] * error + self.ki_list[index] * self.integral[index] + self.kd_list[index] * derivative
        self.previous_error[index] = error

        return output

    def control_pwm(self):
        if self._control_flag:
            for i, adr in enumerate(self.adr):
                self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)
                pid_output = self.compute(self.value_temp[i], i)
                pwm_value = max(0, min(100, pid_output))  # Ensure PWM value is between 0 and 100
                self.io_modbus.io_rpi.aciona_pwm(duty_cycle=pwm_value, saida=adr)
        else:
            pwm_value = 0  # Set PWM to 0 when control flag is False
            for adr in self.adr:
                self.io_modbus.io_rpi.aciona_pwm(duty_cycle=pwm_value, saida=adr)

    def start(self, interval=1):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._run, args=(interval,))
            self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()

    def _run(self, interval):
        while self._running:
            self.control_pwm()
            time.sleep(interval)

    def set_control_flag(self, flag):
        self._control_flag = flag

# Example usage
if __name__ == "__main__":
    io_modbus = IO_MODBUS()

    kp_list = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
    ki_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    kd_list = [0.05, 0.06, 0.07, 0.08, 0.09, 0.1]
    setpoint_list = [50, 55, 60, 65, 70, 75]

    pid_controller = PIDController(kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, setpoint_list=setpoint_list, io_modbus=io_modbus, adr=[1, 2, 3, 4, 5, 6])

    # Start control loop in a separate thread
    pid_controller.start(interval=1)

    # Set control flag to True to start controlling
    pid_controller.set_control_flag(True)

    # Run for 10 seconds then stop
    time.sleep(10)
    pid_controller.set_control_flag(False)
    pid_controller.stop()
