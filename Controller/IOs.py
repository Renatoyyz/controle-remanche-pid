import serial
import time
import random
import threading

class FakeRPiGPIO:
    BCM = "BCM"
    PUD_UP = "PUD_UP"
    IN = "IN"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    def __init__(self):
        self.pins = {}

    def setmode(self, mode):
        self.mode = mode

    def setwarnings(self, state):
        self.warnings = state

    def setup(self, pin, direction, pull_up_down=None):
        self.pins[pin] = {'direction': direction, 'state': self.HIGH if pull_up_down == self.PUD_UP else self.LOW}

    def input(self, pin):
        if pin in self.pins:
            return self.pins[pin]['state']
        raise ValueError(f"Pin {pin} not set up.")

    def output(self, pin, state):
        if pin in self.pins and self.pins[pin]['direction'] == self.OUT:
            self.pins[pin]['state'] = state
        else:
            raise ValueError(f"Pin {pin} not set up or not set as output.")

    def cleanup(self):
        self.pins.clear()


class InOut:
    def __init__(self):
        self.SAIDA_PWM_1 = 20
        self.SAIDA_PWM_2 = 26
        self.SAIDA_PWM_3 = 16
        self.SAIDA_PWM_4 = 19
        self.SAIDA_PWM_5 = 5
        self.SAIDA_PWM_6 = 6

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
        except ImportError:
            print("RPi.GPIO not found. Using fake GPIO.")
            self.GPIO = FakeRPiGPIO()

        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)

        self.GPIO.setup(self.SAIDA_PWM_1,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_2,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_3,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_4,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_5,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_6,  self.GPIO.OUT)

        self.pwm_period = 1.0  # Default period in seconds
        self.pwm_duty_cycles = {
            self.SAIDA_PWM_1: 0,
            self.SAIDA_PWM_2: 0,
            self.SAIDA_PWM_3: 0,
            self.SAIDA_PWM_4: 0,
            self.SAIDA_PWM_5: 0,
            self.SAIDA_PWM_6: 0
        }

        self.pwm_thread = threading.Thread(target=self._pwm_control)
        self.pwm_thread.daemon = True
        self.pwm_thread.start()

    def _pwm_control(self):
        while True:
            for pin, duty_cycle in self.pwm_duty_cycles.items():
                on_time = self.pwm_period * (duty_cycle / 100.0)
                off_time = self.pwm_period - on_time
                if on_time > 0:
                    self.GPIO.output(pin, self.GPIO.LOW)
                    time.sleep(on_time)
                if off_time > 0:
                    self.GPIO.output(pin, self.GPIO.HIGH)
                    time.sleep(off_time)

    def set_pwm_period(self, period):
        self.pwm_period = period

    def set_pwm_duty_cycle(self, pin, duty_cycle):
        if pin in self.pwm_duty_cycles:
            self.pwm_duty_cycles[pin] = duty_cycle

    def aciona_pwm(self, duty_cycle, saida):
        if saida == 1:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_1, duty_cycle)
        elif saida == 2:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_2, duty_cycle)
        elif saida == 3:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_3, duty_cycle)
        elif saida == 4:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_4, duty_cycle)
        elif saida == 5:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_5, duty_cycle)
        elif saida == 6:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_6, duty_cycle)  

class IO_MODBUS:
    def __init__(self, dado=None):
        self.dado = dado

        self.fake_modbus = True
        try:
            self.ser = serial.Serial(
                                        port='/dev/ttyUSB0',  # Porta serial padrão no Raspberry Pi 4
                                        # port='/dev/tty.URT0',  # Porta serial padrão no Raspberry Pi 4
                                        baudrate=9600,       # Taxa de baud
                                        bytesize=8,
                                        parity="N",
                                        stopbits=1,
                                        timeout=1,            # Timeout de leitura
                                        #xonxoff=False,         # Controle de fluxo por software (XON/XOFF)
                                        #rtscts=True
                                    )
        except Exception as e:
            print(f"Erro ao conectar com a serial: {e}")
            return
        self.io_rpi = InOut()

    def crc16_modbus(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if (crc & 0x0001):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def _get_adr_PTA(self):
        broadcast = 0xFF

        id_loc = hex(broadcast)[2:]
        id_loc = id_loc.zfill(2).upper()

        hex_text = f"{id_loc}0300020001"
        bytes_hex = bytes.fromhex(hex_text)  # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC16

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

        for i in range(3):
            try:
                # Repete-se os comandos em decimal com os devidos bytes de CRC
                self.ser.write([broadcast,3,0,2,0,1,parte_inferior,parte_superior])
                # self.ser.flush()
                # start_time = time.time()

                while not self.ser.readable():
                    # if time.time() - start_time > self.ser.timeout:
                    #     print("Timeout: Nenhuma resposta do escravo.")
                    #     break
                    time.sleep(0.1)  # Aguarde um curto período antes de verificar novamente

                dados_recebidos = self.ser.read(7)
                self.ser.flushInput()  # Limpa o buffer de entrada após a leitura
                if dados_recebidos != b'':
                    dados_recebidos = dados_recebidos.hex()
                    hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]
                    bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
                    crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

                    parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
                    parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

                    superior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int
                    inferior_crc = int(dados_recebidos[10:12],16) # Transforma de hexa para int

                    if parte_superior == superior_crc and parte_inferior == inferior_crc:
                        dados_recebidos = dados_recebidos[6:10]
                        dados_recebidos = int(dados_recebidos,16)
                        return dados_recebidos
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print(f"Erro de comunicação: {e}")
                return -1 # Indica erro de alguma natureza....
        return -1

    def config_adr_PTA(self, adr):
        id_loc = hex(adr)[2:]
        id_loc = id_loc.zfill(2).upper()
        
    # def wp_8026(self, adr, input):
    #     if self.fake_modbus == False:
    #         pass
    #         # return self.wp_8026_(adr=adr, input=input)
    #     else:
    #         # return random.randint(0,1)
    #         return self.dado.passa_condutividade  if input == 8 else self.dado.passa_isolacao
        
    def get_temperature_channel(self, adr):
        if self.fake_modbus == False:
            return 0
    
    def reset_serial(self):
        try:
            self.ser.close()
            time.sleep(0.5)  # Aguarda um curto período antes de reabrir a porta
            self.ser.open()
            self.ser.flushInput()  # Limpa o buffer de entrada após reabrir a porta
            print("Porta serial resetada com sucesso.")
        except Exception as e:
            print(f"Erro ao resetar a porta serial: {e}")

if __name__ == '__main__':
    import time
    io = IO_MODBUS()
    io._get_adr_PTA()