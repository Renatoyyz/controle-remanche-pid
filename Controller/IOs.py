import serial
import time
import random

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
        self.SAIDA_PWM = 10
        self.SINALEIRO_VERDE = 13
        self.SINALEIRO_VERMELHO = 19
        self.BOTAO_EMERGENCIA = 23
        self.BOT_ACIO_E = 22
        self.BOT_ACIO_D = 24

        self.RELE_4 = 16

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
        except ImportError:
            print("RPi.GPIO not found. Using fake GPIO.")
            self.GPIO = FakeRPiGPIO()

        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)

        self.GPIO.setup(self.BOT_ACIO_E, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
        self.GPIO.setup(self.BOT_ACIO_D, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)


        self.GPIO.setup(self.BOTAO_EMERGENCIA, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)

        self.GPIO.setup(self.SAIDA_PWM,  self.GPIO.OUT)
        self.GPIO.setup(self.RELE_4, self.GPIO.OUT)
        self.GPIO.setup(self.SINALEIRO_VERDE, self.GPIO.OUT)
        self.GPIO.setup(self.SINALEIRO_VERMELHO, self.GPIO.OUT)

    @property
    # off_app
    def bot_acio_e(self):
        return self.GPIO.input(self.BOT_ACIO_E)

    @property
    def bot_acio_d(self):
        return self.GPIO.input(self.BOT_ACIO_D)

    @property
    def bot_emergencia(self):
        return self.GPIO.input(self.BOTAO_EMERGENCIA)
    
    def aciona_rele_4(self, status):
        if status == 1:
            self.GPIO.output(self.RELE_4,1)
        else:
            self.GPIO.output(self.RELE_4,0)

    def sinaleiro_verde(self):
        self.GPIO.output(self.SINALEIRO_VERMELHO, 1)
        self.GPIO.output(self.SINALEIRO_VERDE, 0)

    def sinaleiro_vermelho(self):
        self.GPIO.output(self.SINALEIRO_VERDE, 1)
        self.GPIO.output(self.SINALEIRO_VERMELHO, 0)

    def desliga_torre(self):
        self.GPIO.output(self.SINALEIRO_VERDE, 1)
        self.GPIO.output(self.SINALEIRO_VERMELHO, 1)

    def aciona_pwm(self, status):
        if status == 1:
            self.GPIO.output(self.SAIDA_PWM,1)
        else:
            self.GPIO.output(self.SAIDA_PWM,0)
        
class IO_MODBUS:
    def __init__(self, dado=None):

        self.io_rpi = InOut()
        self.ADR_1 = 1 # Endereço do WP8027 dos relés do lado esquerdo - 16 saídas
        self.ADR_2 = 2 # Endereço do WP8027 dos relés do lado direito - 16 saídas
        self.ADR_3 = 3 # Endereço do WP8027 de automações em geral - 16 saídas
        self.ADR_4 = 4 # Endereço do WP8026 de automações em geral - 16 entradas

        self.valor_saida_direito = 0
        self.valor_saida_esquerdo = 0
        self.valor_saida_geral = 0
        self.valor_saida_geral2 = 0

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

    def wp_8027(self, adr, out, on_off):
        if self.fake_modbus == False:
            pass
            # return self.wp_8027_(adr=adr, out=out,on_off=on_off)
        else:
            return -1
        
    def wp_8026(self, adr, input):
        if self.fake_modbus == False:
            pass
            # return self.wp_8026_(adr=adr, input=input)
        else:
            # return random.randint(0,1)
            return self.dado.passa_condutividade  if input == 8 else self.dado.passa_isolacao
        
    def get_temperature(self, adr):
        if self.fake_modbus == False:
            pass
    
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
    cmd = ""
    while cmd != "q":
        adr = input("Digite o endereço\n")
        cmd = input("Digite a saida que queira testar.\n")
        stat = input("Digite 1=ligar 0=desligar.\n")
        try:
            io.wp_8027(int(adr),int(cmd),int(stat))
            print(f"A saida {int(cmd)} foi acionada.")
        except:
            if cmd != "q":
                print("Digite um número válido.")
        time.sleep(1)