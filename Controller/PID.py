import threading
import time
from IOs import IO_MODBUS

class PIDController:
    def __init__(self, kp, ki, kd, setpoint, io_modbus, adr):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.io_modbus = io_modbus
        self.adr = adr
        self.integral = 0
        self.previous_error = 0
        self._running = False
        self._control_flag = False
        self._thread = None

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.previous_error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        return output

    def control_pwm(self):
        if self._control_flag:
            current_value = self.io_modbus.get_temperature(self.adr)
            pid_output = self.compute(current_value)
            pwm_value = max(0, min(100, pid_output))  # Ensure PWM value is between 0 and 100
        else:
            pwm_value = 0  # Set PWM to 0 when control flag is False
        self.io_modbus.io_rpi.aciona_pwm(pwm_value)

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

    # Parameters for heating a copper piece with a 100W resistor
    # Assuming specific heat capacity of copper is 0.385 J/g°C and density is 8.96 g/cm³
    volume_cm3 = 4170 * 8  # Volume in cubic millimeters
    volume_cm3 /= 1000  # Convert to cubic centimeters
    mass_g = volume_cm3 * 8.96  # Mass in grams

    # Energy required to heat the copper piece (Q = mcΔT)
    specific_heat_capacity = 0.385  # J/g°C
    temperature_increase = 50  # Desired temperature increase in °C
    energy_required_joules = mass_g * specific_heat_capacity * temperature_increase

    # Power of the resistor
    power_watts = 100  # 100W resistor

    # Time to heat the copper piece (t = Q / P)
    time_seconds = energy_required_joules / power_watts

    # Convert time to minutes
    time_minutes = time_seconds / 60

    # PID parameters (these values might need to be tuned experimentally)
    kp = 1.0
    ki = 0.1
    kd = 0.05

    pid_controller = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=50, io_modbus=io_modbus, adr=1)

    # Start control loop in a separate thread
    pid_controller.start(interval=1)

    # Set control flag to True to start controlling
    pid_controller.set_control_flag(True)

    # Run for 10 seconds then stop
    time.sleep(10)
    pid_controller.set_control_flag(False)
    pid_controller.stop()