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
        self.command_queue = Queue(maxsize=20)  # Limita tamanho da fila
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
        try:
            # Tenta adicionar, mas não bloqueia se a fila estiver cheia
            self.command_queue.put({
                'type': 'display_string',
                'string': string,
                'line': line,
                'pos': pos
            }, block=False)
        except:
            # Se a fila estiver cheia, ignora (evita travar o programa)
            pass
        
    def clear(self):
        """Enfileira um comando de limpeza"""
        try:
            self.command_queue.put({'type': 'clear'}, block=False)
        except:
            pass
        
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
        
        # Variáveis de controle para atualização condicional
        last_tela = -1
        last_counter = -1
        last_temp_values = [0] * 6
        needs_full_redraw = True
        blink_counter = 0  # Contador para efeito de piscar
        blink_state = True  # Estado do caractere piscante
        
        while True:
            try:
                # Delay para reduzir sobrecarga do LCD e melhorar resposta do encoder
                time.sleep(0.05)
                
                if dado.telas == dado.TELA_INICIAL:
                    # Redesenha apenas se mudou de tela
                    if last_tela != dado.TELA_INICIAL:
                        lcd_thread.display_string("**** QUALIFIX **** ", 1, 1)
                        lcd_thread.display_string("Iniciar", 2, 1)
                        lcd_thread.display_string("Configuracoes", 3, 1)
                        last_tela = dado.TELA_INICIAL
                        last_counter = -1  # Força atualização do cursor
                    
                    # Atualiza cursor apenas se mudou
                    if last_counter != pot.counter:
                        if pot.counter == 1:
                            lcd_thread.display_string(">", 2, 0)
                            lcd_thread.display_string(" ", 3, 0)
                        elif pot.counter == 2:
                            lcd_thread.display_string(">", 3, 0)
                            lcd_thread.display_string(" ", 2, 0)
                        last_counter = pot.counter
                    if (pot.get_sw_status == 0 and (pot.counter == 1 )) or (pot.counter == 1 and io.io_rpi.get_aciona_maquina == 1):
                        lcd_thread.clear()
                        lcd_thread.display_string("**** AGUARDE ****  ", 1, 1)
                        dado.set_telas(dado.TELA_EXECUCAO)
                        pid.set_control_flag(True)
                        lcd_thread.clear()
                        last_tela = -1  # Força redesenho
                        time.sleep(0.3)
                    elif pot.get_sw_status == 0 and pot.counter == 2:
                        pot.counter = 1
                        dado.set_telas(dado.TELA_CONFIGURACAO)
                        lcd_thread.clear()
                        last_tela = -1  # Força redesenho
                        time.sleep(0.3)

                elif dado.telas == dado.TELA_EXECUCAO:
                    # Atualiza contador de piscar (a cada 10 ciclos = ~500ms)
                    blink_counter += 1
                    if blink_counter >= 10:
                        blink_counter = 0
                        blink_state = not blink_state
                    
                    # Redesenha título com caractere piscante
                    if last_tela != dado.TELA_EXECUCAO:
                        blink_char = "*" if blink_state else " "
                        lcd_thread.display_string(f"{blink_char}Execucao          ", 1, 0)
                        last_tela = dado.TELA_EXECUCAO
                    else:
                        # Atualiza apenas o caractere piscante (posição 0)
                        if blink_counter == 0:  # Só atualiza quando muda de estado
                            blink_char = "*" if blink_state else " "
                            lcd_thread.display_string(blink_char, 1, 0)
                    
                    # Atualiza temperaturas apenas se mudaram (tolerância de 1°C)
                    temp_changed = False
                    for i in range(6):
                        if abs(pid.value_temp[i] - last_temp_values[i]) >= 1:
                            temp_changed = True
                            break
                    
                    if temp_changed:
                        lcd_thread.display_string(f"1:{pid.value_temp[0]:.0f} 4:{pid.value_temp[3]:.0f}     ", 2, 1)
                        lcd_thread.display_string(f"2:{pid.value_temp[1]:.0f} 5:{pid.value_temp[4]:.0f}     ", 3, 1)
                        lcd_thread.display_string(f"3:{pid.value_temp[2]:.0f} 6:{pid.value_temp[5]:.0f}     ", 4, 1)
                        last_temp_values = pid.value_temp.copy()

                    if pot.get_sw_status == 0 or io.io_rpi.get_aciona_maquina == 1:
                        lcd_thread.clear()
                        lcd_thread.display_string("**** AGUARDE ****  ", 1, 1)
                        io.io_rpi.aciona_maquina_pronta(False) # Desliga a saida que habilita a prensa
                        dado.set_telas(dado.TELA_INICIAL)
                        pid.set_control_flag(False)
                        lcd_thread.clear()
                        pot.counter = 1
                        last_tela = -1  # Força redesenho
                        blink_counter = 0  # Reseta contador de piscar
                        blink_state = True  # Reseta estado de piscar
                        time.sleep(0.3)

                elif dado.telas == dado.TELA_CONFIGURACAO:
                    pot.val_max = 3
                    
                    # Redesenha apenas se mudou de tela
                    if last_tela != dado.TELA_CONFIGURACAO:
                        lcd_thread.display_string("Configurar:", 1, 1)
                        lcd_thread.display_string("Temp", 2, 1)
                        lcd_thread.display_string("PID", 3, 1)
                        lcd_thread.display_string("Sair", 4, 1)
                        last_tela = dado.TELA_CONFIGURACAO
                        last_counter = -1  # Força atualização do cursor
                    
                    # Atualiza cursor apenas se mudou
                    if last_counter != pot.get_counter():
                        if pot.get_counter() == 1:
                            lcd_thread.display_string(">", 2, 0)
                            lcd_thread.display_string(" ", 3, 0)
                            lcd_thread.display_string(" ", 4, 0)
                        elif pot.get_counter() == 2:
                            lcd_thread.display_string(" ", 2, 0)
                            lcd_thread.display_string(">", 3, 0)
                            lcd_thread.display_string(" ", 4, 0)
                        elif pot.get_counter() == 3:
                            lcd_thread.display_string(" ", 2, 0)
                            lcd_thread.display_string(" ", 3, 0)
                            lcd_thread.display_string(">", 4, 0)
                        last_counter = pot.get_counter()
                    
                    if pot.get_sw_status == 0:
                        if pot.get_counter() == 1:
                            dado.set_telas(TELA_CONFIGURACAO_TEMP)
                            lcd_thread.clear()
                            last_tela = -1
                            time.sleep(0.3)
                        elif pot.get_counter() == 2:
                            dado.set_telas(TELA_CONFIGURACAO_PID)
                            lcd_thread.clear()
                            last_tela = -1
                            time.sleep(0.3)
                        elif pot.get_counter() == 3:
                            dado.set_telas(dado.TELA_INICIAL)
                            lcd_thread.clear()
                            pot.counter = 1
                            last_tela = -1
                            time.sleep(0.3)

                elif dado.telas == TELA_CONFIGURACAO_TEMP:
                    pot.val_max = 6  # Limita a quantidade de canais para ajuste de setpoint
                    canal = pot.get_counter()
                    
                    # Redesenha apenas se mudou de tela ou canal
                    if last_tela != TELA_CONFIGURACAO_TEMP or last_counter != canal:
                        lcd_thread.display_string(f"Canal {canal}          ", 1, 1)
                        lcd_thread.display_string(f"Temp: {setpoint_list[canal-1]}C   ", 2, 1)
                        lcd_thread.display_string("Sair:             ", 3, 1)
                        last_tela = TELA_CONFIGURACAO_TEMP
                        last_counter = canal

                    if pot.get_sw_status == 0:
                        time.sleep(0.6)
                        ajt = 1
                        pot.val_max = 300  # Limita o ajuste de temperatura
                        pot.counter = setpoint_list[canal-1]
                        last_setpoint = -1
                        while ajt == 1:
                            time.sleep(0.05)  # Delay para reduzir atualizações
                            current_setpoint = pot.get_counter()
                            if current_setpoint != last_setpoint:
                                setpoint_list[canal-1] = current_setpoint
                                lcd_thread.display_string(f"Temp: {setpoint_list[canal-1]}C   ", 2, 1)
                                last_setpoint = current_setpoint
                            if pot.get_sw_status == 0:
                                save_setpoint_to_file(setpoint_list)  # Salva os setpoints ajustados
                                pid.setpoint_list = setpoint_list  # Atualiza os setpoints no controlador PID
                                ajt = 0
                                pot.val_max = 6
                                pot.counter = 1
                                dado.set_telas(dado.TELA_CONFIGURACAO)
                                lcd_thread.clear()
                                last_tela = -1
                                time.sleep(0.3)

                elif dado.telas == TELA_CONFIGURACAO_PID:
                    pot.val_max = 6  # Limita a quantidade de canais para ajuste de PID
                    canal = pot.get_counter()
                    
                    # Redesenha apenas se mudou de tela ou canal
                    if last_tela != TELA_CONFIGURACAO_PID or last_counter != canal:
                        lcd_thread.display_string("Ajuste PID        ", 1, 1)
                        lcd_thread.display_string(f"Canal {canal}          ", 2, 1)
                        lcd_thread.display_string("Kp: {:.2f} Ki: {:.2f}  ".format(kp_list[canal-1], ki_list[canal-1]), 3, 1)
                        lcd_thread.display_string("Kd: {:.2f}        ".format(kd_list[canal-1]), 4, 1)
                        last_tela = TELA_CONFIGURACAO_PID
                        last_counter = canal

                    if pot.get_sw_status == 0:
                        time.sleep(0.6)
                        pot.val_max = 4000  # Limita o ajuste de PID
                        ajt = 1
                        lcd_thread.clear()
                        pot.counter = int(kp_list[canal-1] * 100)
                        last_kp = -1
                        while ajt == 1:
                            time.sleep(0.05)  # Delay para reduzir atualizações
                            current_kp = pot.get_counter() / 100.0
                            if abs(current_kp - last_kp) >= 0.01:
                                lcd_thread.display_string("Ajuste Kp         ", 1, 1)
                                lcd_thread.display_string(f"Kp: {current_kp:.2f}        ", 2, 1)
                                kp_list[canal-1] = current_kp
                                last_kp = current_kp
                            if pot.get_sw_status == 0:
                                time.sleep(0.6)
                                ajt = 2
                                lcd_thread.clear()
                                pot.counter = int(ki_list[canal-1] * 100)
                                last_ki = -1
                                while ajt == 2:
                                    time.sleep(0.05)  # Delay para reduzir atualizações
                                    current_ki = pot.get_counter() / 100.0
                                    if abs(current_ki - last_ki) >= 0.01:
                                        lcd_thread.display_string("Ajuste Ki         ", 1, 1)
                                        lcd_thread.display_string(f"Ki: {current_ki:.2f}        ", 2, 1)
                                        ki_list[canal-1] = current_ki
                                        last_ki = current_ki
                                    if pot.get_sw_status == 0:
                                        time.sleep(0.6)
                                        ajt = 3
                                        lcd_thread.clear()
                                        pot.counter = int(kd_list[canal-1] * 100)
                                        last_kd = -1
                                        while ajt == 3:
                                            time.sleep(0.05)  # Delay para reduzir atualizações
                                            current_kd = pot.get_counter() / 100.0
                                            if abs(current_kd - last_kd) >= 0.01:
                                                lcd_thread.display_string("Ajuste Kd         ", 1, 1)
                                                lcd_thread.display_string(f"Kd: {current_kd:.2f}        ", 2, 1)
                                                kd_list[canal-1] = current_kd
                                                last_kd = current_kd
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
                                                last_tela = -1
                                                time.sleep(0.3)
                
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