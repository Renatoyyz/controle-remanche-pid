import time
import threading
from queue import Queue, Empty
from Controller.PID import PIDController
from Controller.IOs import IO_MODBUS, InOut
from Controller.Dados import Dado
from Controller.Lcd import Lcd
from Controller.KY040 import KY040
import os
import json

def save_setpoint_to_file(setpoint_list, filename="setpoint_list.json"):
    """
    Salva os setpoints de cada canal em um arquivo JSON.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Salva a lista de setpoints no arquivo JSON
    with open(file_path, "w") as file:
        json.dump(setpoint_list, file)

def read_setpoint_from_file(filename="setpoint_list.json"):
    """
    Lê os setpoints de cada canal de um arquivo JSON.
    Se o arquivo não existir, cria um com valores padrão.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Valores padrão caso o arquivo não exista
    default_setpoint_list = [50, 50, 50, 50, 50, 50]

    if not os.path.exists(file_path):
        # Se o arquivo não existir, cria um com valores padrão
        with open(file_path, "w") as file:
            json.dump(default_setpoint_list, file)
        return default_setpoint_list

    # Se o arquivo existir, carrega os valores
    with open(file_path, "r") as file:
        setpoint_list = json.load(file)

    return setpoint_list

def save_pid_values(kp_list, ki_list, kd_list, filename="pid_values.json"):
    """
    Salva os valores de Kp, Ki e Kd em um arquivo JSON.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Cria um dicionário com os valores
    pid_values = {
        "kp": kp_list,
        "ki": ki_list,
        "kd": kd_list
    }

    # Salva o dicionário no arquivo JSON
    with open(file_path, "w") as file:
        json.dump(pid_values, file)

def load_pid_values(filename="pid_values.json"):
    """
    Carrega os valores de Kp, Ki e Kd de um arquivo JSON.
    Se o arquivo não existir, cria um com valores padrão.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Valores padrão caso o arquivo não exista
    default_kp = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
    default_ki = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    default_kd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if not os.path.exists(file_path):
        # Se o arquivo não existir, cria um com valores padrão
        pid_values = {
            "kp": default_kp,
            "ki": default_ki,
            "kd": default_kd
        }
        with open(file_path, "w") as file:
            json.dump(pid_values, file)
        return default_kp, default_ki, default_kd

    # Se o arquivo existir, carrega os valores
    with open(file_path, "r") as file:
        pid_values = json.load(file)

    return pid_values["kp"], pid_values["ki"], pid_values["kd"]


class LcdThread(threading.Thread):
    """
    Thread separada para gerenciar o LCD.
    Evita que problemas I2C travem o programa principal.
    """
    def __init__(self, lcd_obj):
        super().__init__(daemon=True)
        self.lcd_obj = lcd_obj
        self.command_queue = Queue()
        self.running = True
        
    def run(self):
        """Loop principal da thread LCD"""
        while self.running:
            try:
                # Tenta pegar comando com timeout
                command = self.command_queue.get(timeout=0.5)
                
                try:
                    if command['type'] == 'display_string':
                        self.lcd_obj.lcd_display_string(
                            command['string'],
                            command['line'],
                            command['pos']
                        )
                    elif command['type'] == 'clear':
                        self.lcd_obj.lcd_clear()
                    elif command['type'] == 'display_string_inverter':
                        self.lcd_obj.lcd_display_string_inverter(
                            command['string'],
                            command['line'],
                            command['pos']
                        )
                    elif command['type'] == 'backlight':
                        self.lcd_obj.backlight(command['state'])
                except Exception as e:
                    print(f"Erro ao executar comando LCD: {e}")
                    
            except Empty:
                # Sem comando, continua aguardando
                continue
            except Exception as e:
                print(f"Erro na thread LCD: {e}")
                
    def display_string(self, string, line=1, pos=0):
        """Enfileira um comando de exibição de string"""
        self.command_queue.put({
            'type': 'display_string',
            'string': string,
            'line': line,
            'pos': pos
        })
        
    def clear(self):
        """Enfileira um comando de limpeza"""
        self.command_queue.put({'type': 'clear'})
        
    def display_string_inverter(self, string, line=1, pos=0):
        """Enfileira um comando de exibição invertida"""
        self.command_queue.put({
            'type': 'display_string_inverter',
            'string': string,
            'line': line,
            'pos': pos
        })
        
    def backlight(self, state):
        """Enfileira um comando de controle de backlight"""
        self.command_queue.put({
            'type': 'backlight',
            'state': state
        })
        
    def stop(self):
        """Para a thread"""
        self.running = False
        self.join(timeout=2)

if __name__ == "__main__":
    setpoint_list = read_setpoint_from_file()
    kp_list, ki_list, kd_list = load_pid_values()

    dado = Dado()
    lcd = Lcd()
    lcd_thread = LcdThread(lcd)  # ✅ NOVO: Thread separada para LCD
    lcd_thread.start()
    
    io = IO_MODBUS(dado=dado)
    pot = KY040(io=io, val_min=1, val_max=2)
    pid = PIDController(setpoint_list=setpoint_list, io_modbus=io, kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, adr=[1, 2, 3, 4, 5, 6])
    pid.start(interval=0.5)

    TELA_CONFIGURACAO_PID = 3
    TELA_CONFIGURACAO_TEMP = 4

    try:
        pot.counter = 1
        pot.val_max = 2
        while True:
            try:
                if dado.telas == dado.TELA_INICIAL:
                    lcd_thread.display_string("**** QUALIFIX **** ", 1, 1)
                    lcd_thread.display_string("Iniciar", 2, 1)
                    lcd_thread.display_string("Configuracoes", 3, 1)
                    if pot.counter == 1:
                        lcd_thread.display_string(">", 2, 0)
                        lcd_thread.display_string(" ", 3, 0)
                    elif pot.counter == 2:
                        lcd_thread.display_string(">", 3, 0)
                        lcd_thread.display_string(" ", 2, 0)
                    if (pot.get_sw_status == 0 and (pot.counter == 1 )) or (pot.counter == 1 and io.io_rpi.get_aciona_maquina == 1):
                        lcd_thread.clear()
                        lcd_thread.display_string("**** AGUARDE ****  ", 1, 1)
                        dado.set_telas(dado.TELA_EXECUCAO)
                        pid.set_control_flag(True)
                        lcd_thread.clear()
                        time.sleep(0.3)
                    elif pot.get_sw_status == 0 and pot.counter == 2:
                        pot.counter = 1
                        dado.set_telas(dado.TELA_CONFIGURACAO)
                        lcd_thread.clear()
                        time.sleep(0.3)

                elif dado.telas == dado.TELA_EXECUCAO:
                    lcd_thread.display_string("Execucao          ", 1, 1)
                    lcd_thread.display_string(f"1:{pid.value_temp[0]} 4:{pid.value_temp[3]}", 2, 1)
                    lcd_thread.display_string(f"2:{pid.value_temp[1]} 5:{pid.value_temp[4]}", 3, 1)
                    lcd_thread.display_string(f"3:{pid.value_temp[2]} 6:{pid.value_temp[5]}", 4, 1)

                    if pot.get_sw_status == 0 or io.io_rpi.get_aciona_maquina == 1:
                        lcd_thread.clear()
                        lcd_thread.display_string("**** AGUARDE ****  ", 1, 1)
                        io.io_rpi.aciona_maquina_pronta(False) # Desliga a saida que habilita a prensa
                        dado.set_telas(dado.TELA_INICIAL)
                        pid.set_control_flag(False)
                        lcd_thread.clear()
                        pot.counter = 1
                        time.sleep(0.3)

                elif dado.telas == dado.TELA_CONFIGURACAO:
                    pot.val_max = 3
                    lcd_thread.display_string("Configurar:", 1, 1)
                    lcd_thread.display_string("Temp", 2, 1)
                    lcd_thread.display_string("PID", 3, 1)
                    lcd_thread.display_string("Sair", 4, 1)
                    if pot.get_counter() == 1:
                        lcd_thread.display_string(">", 2, 0)
                        lcd_thread.display_string(" ", 3, 0)
                        lcd_thread.display_string(" ", 4, 0)
                        if pot.get_sw_status == 0:
                            dado.set_telas(TELA_CONFIGURACAO_TEMP)
                            lcd_thread.clear()
                            time.sleep(0.3)
                    elif pot.get_counter() == 2:
                        lcd_thread.display_string(" ", 2, 0)
                        lcd_thread.display_string(">", 3, 0)
                        lcd_thread.display_string(" ", 4, 0)
                        if pot.get_sw_status == 0:
                            dado.set_telas(TELA_CONFIGURACAO_PID)
                            lcd_thread.clear()
                            time.sleep(0.3)
                    elif pot.get_counter() == 3:
                        lcd_thread.display_string(" ", 2, 0)
                        lcd_thread.display_string(" ", 3, 0)
                        lcd_thread.display_string(">", 4, 0)
                        if pot.get_sw_status == 0:
                            dado.set_telas(dado.TELA_INICIAL)
                            lcd_thread.clear()
                            pot.counter = 1
                            time.sleep(0.3)

                elif dado.telas == TELA_CONFIGURACAO_TEMP:
                    pot.val_max = 6  # Limita a quantidade de canais para ajuste de setpoint
                    canal = pot.get_counter()
                    lcd_thread.display_string(f"Canal {canal}", 1, 1)
                    lcd_thread.display_string(f"Temp: {setpoint_list[canal-1]}C", 2, 1)
                    lcd_thread.display_string("Sair: ", 3, 1)

                    if pot.get_sw_status == 0:
                        time.sleep(0.6)
                        ajt = 1
                        pot.val_max = 300  # Limita o ajuste de temperatura
                        pot.counter = setpoint_list[canal-1]
                        while ajt == 1:
                            setpoint_list[canal-1] = pot.get_counter()
                            lcd_thread.display_string(f"Temp: {setpoint_list[canal-1]}C", 2, 1)
                            if pot.get_sw_status == 0:
                                save_setpoint_to_file(setpoint_list)  # Salva os setpoints ajustados
                                pid.setpoint_list = setpoint_list  # Atualiza os setpoints no controlador PID
                                ajt = 0
                                pot.val_max = 6
                                pot.counter = 1
                                dado.set_telas(dado.TELA_CONFIGURACAO)
                                lcd_thread.clear()
                                time.sleep(0.3)

                elif dado.telas == TELA_CONFIGURACAO_PID:
                    pot.val_max = 6  # Limita a quantidade de canais para ajuste de PID
                    canal = pot.get_counter()
                    lcd_thread.display_string("Ajuste PID", 1, 1)
                    lcd_thread.display_string(f"Canal {canal}", 2, 1)
                    lcd_thread.display_string("Kp: {:.2f} Ki: {:.2f}".format(kp_list[canal-1], ki_list[canal-1]), 3, 1)
                    lcd_thread.display_string("Kd: {:.2f}".format(kd_list[canal-1]), 4, 1)

                    if pot.get_sw_status == 0:
                        time.sleep(0.6)
                        pot.val_max = 4000  # Limita o ajuste de PID
                        ajt = 1
                        lcd_thread.clear()
                        pot.counter = kp_list[canal-1] * 100
                        while ajt == 1:
                            lcd_thread.display_string("Ajuste Kp", 1, 1)
                            lcd_thread.display_string(f"Kp: {kp_list[canal-1]:.2f}", 2, 1)
                            kp_list[canal-1] = pot.get_counter() / 100.0
                            if pot.get_sw_status == 0:
                                time.sleep(0.6)
                                ajt = 2
                                lcd_thread.clear()
                                pot.counter = ki_list[canal-1] * 100
                                while ajt == 2:
                                    lcd_thread.display_string("Ajuste Ki", 1, 1)
                                    lcd_thread.display_string(f"Ki: {ki_list[canal-1]:.2f}", 2, 1)
                                    ki_list[canal-1] = pot.get_counter() / 100.0
                                    if pot.get_sw_status == 0:
                                        time.sleep(0.6)
                                        ajt = 3
                                        lcd_thread.clear()
                                        pot.counter = kd_list[canal-1] * 100
                                        while ajt == 3:
                                            lcd_thread.display_string("Ajuste Kd", 1, 1)
                                            lcd_thread.display_string(f"Kd: {kd_list[canal-1]:.2f}", 2, 1)
                                            kd_list[canal-1] = pot.get_counter() / 100.0
                                            if pot.get_sw_status == 0:
                                                ajt = 0
                                                pot.val_max = 6  # Limita a quantidade de canais para ajuste de PID
                                                pot.counter = 1
                                                save_pid_values(kp_list, ki_list, kd_list)  # Salva os valores ajustados
                                                pid.kp_list = kp_list
                                                pid.ki_list = ki_list
                                                pid.kd_list = kd_list
                                                dado.set_telas(dado.TELA_CONFIGURACAO)
                                                lcd_thread.clear()
                                                time.sleep(0.3)
                
                # Pequeno delay para não sobrecarregar o CPU
                time.sleep(0.05)
                
            except Exception as e:
                print(f"Erro no loop principal: {e}")
                time.sleep(0.1)

    except KeyboardInterrupt:
        lcd_thread.clear()
        lcd_thread.stop()  # Para a thread LCD corretamente
        pot.cleanup()
        io.io_rpi.cleanup()
        pid.set_control_flag(False)
        pid.stop()
        print("Saindo do programa")
        exit()