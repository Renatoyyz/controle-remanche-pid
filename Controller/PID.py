import threading
import time
from Controller.IOs import IO_MODBUS

class PIDController:
    def __init__(self, kp_list=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], ki_list=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5], kd_list=[0.05, 0.05, 0.05, 0.05, 0.05, 0.05], setpoint_list=[180, 180, 180, 180, 180, 180], io_modbus=None, adr=[1, 2, 3, 4, 5, 6]):
        self.kp_list = kp_list
        self.ki_list = ki_list
        self.kd_list = kd_list
        self.setpoint_list = setpoint_list
        self.value_temp = [0, 0, 0, 0, 0, 0]

        self.io_modbus = io_modbus
        self.adr = adr
        self.integral = [0] * len(adr)
        self.previous_error = [0] * len(adr)
        self.previous_time = [time.time()] * len(adr)
        self._running = False
        self._control_flag = False
        self._thread = None
        
        # Limites para anti-windup
        self.output_min = 0
        self.output_max = 100
        self.integral_min = [-50] * len(adr)  # Limites do termo integral
        self.integral_max = [50] * len(adr)

    def compute(self, current_value, index):
        current_time = time.time()
        dt = current_time - self.previous_time[index]
        
        if dt <= 0:
            dt = 0.001  # Evita divisão por zero
        
        # Controle PID direto com o setpoint final
        error = self.setpoint_list[index] - current_value
        
        # Termo Proporcional
        proportional = self.kp_list[index] * error
        
        # Termo Integral com anti-windup
        self.integral[index] += error * dt
        # Limita o termo integral para evitar windup
        self.integral[index] = max(self.integral_min[index], min(self.integral_max[index], self.integral[index]))
        integral_term = self.ki_list[index] * self.integral[index]
        
        # Termo Derivativo
        derivative = (error - self.previous_error[index]) / dt
        derivative_term = self.kd_list[index] * derivative
        
        # Saída PID
        output = proportional + integral_term + derivative_term
        
        # Limita a saída e implementa anti-windup
        output_limited = max(self.output_min, min(self.output_max, output))
        
        # Anti-windup: se a saída está saturada, não acumula mais no integral
        if output != output_limited:
            # Remove o excesso do integral
            if output > output_limited and error > 0:
                self.integral[index] -= error * dt
            elif output < output_limited and error < 0:
                self.integral[index] -= error * dt
        
        # Atualiza valores anteriores
        self.previous_error[index] = error
        self.previous_time[index] = current_time

        return output_limited

    def control_pwm(self):
        if self._control_flag:
            for i, adr in enumerate(self.adr):
                self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)
                
                pid_output = self.compute(self.value_temp[i], i)
                pwm_value = pid_output  # A saída já está limitada na função compute
                self.io_modbus.io_rpi.aciona_pwm(duty_cycle=pwm_value, saida=adr)

                # Verifica se todos os canais atingiram o setpoint dentro da faixa permitida
                all_channels_ready = True
                for j, setpoint in enumerate(self.setpoint_list):
                    if not (setpoint * 0.92 <= self.value_temp[j] <= setpoint * 1.08):  # Faixa de 92% a 108% do setpoint
                        all_channels_ready = False
                        break

                # Aciona a saída de máquina pronta se todos os canais estiverem prontos
                if all_channels_ready:
                    self.io_modbus.io_rpi.aciona_maquina_pronta(False)
                else:
                    self.io_modbus.io_rpi.aciona_maquina_pronta(True)
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

        # Atualiza as temperaturas atuais antes de verificar o flag
        for i, adr in enumerate(self.adr):
            self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)

        if not flag:
            # Resetar estados internos ao desativar o controle
            self.integral = [0] * len(self.adr)
            self.previous_error = [0] * len(self.adr)
            self.previous_time = [time.time()] * len(self.adr)

# Example usage
if __name__ == "__main__":
    io_modbus = IO_MODBUS()

    kp_list = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
    ki_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    kd_list = [0.05, 0.06, 0.07, 0.08, 0.09, 0.1]
    setpoint_list = [180, 180, 180, 180, 180, 180]

    pid_controller = PIDController(kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, setpoint_list=setpoint_list, io_modbus=io_modbus, adr=[1, 2, 3, 4, 5, 6])

    # Start control loop in a separate thread
    pid_controller.start(interval=1)

    # Activate control
    pid_controller.set_control_flag(True)

    # Run for the duration of the control process
    time.sleep(300)  # Example: wait for 5 minutes

    # Deactivate control
    pid_controller.set_control_flag(False)

    # Stop the control loop
    pid_controller.stop()