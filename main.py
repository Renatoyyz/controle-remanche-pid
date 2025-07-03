import time
from Controller.PID import PIDController
from Controller.IOs import IO_MODBUS
from Controller.Dados import Dado
from Controller.Lcd import Lcd
from Controller.KY040 import KY040
import os
import json

# kp_list = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
# ki_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
# kd_list = [0.05, 0.06, 0.07, 0.08, 0.09, 0.1]
# setpoint_list = [50, 55, 60, 65, 70, 75]
def save_setpoint_to_file(setpoint, filename="setpoint.txt"):
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)
    with open(file_path, "w") as file:
        file.write(str(setpoint))

def read_setpoint_from_file(filename="setpoint.txt"):
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)
    if not os.path.exists(file_path):
        with open(file_path, "w") as file:
            file.write("50")  # Default setpoint value
    with open(file_path, "r") as file:
        setpoint = file.read()
    return int(setpoint)

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

if __name__ == "__main__":
    setpoint = read_setpoint_from_file()
    kp_list, ki_list, kd_list = load_pid_values()  # Carrega os valores salvos
    setpoint_list = [setpoint, setpoint, setpoint, setpoint, setpoint, setpoint]

    dado = Dado()
    lcd = Lcd()
    io = IO_MODBUS( dado=dado)
    pot = KY040( io=io, val_min=1, val_max=2)
    pid = PIDController(setpoint_list=setpoint_list, io_modbus=io, kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, adr=[1, 2, 3, 4, 5, 6])
    pid.start(interval=0.5)
    

        # Adicione uma nova constante para a tela de configuração PID
    TELA_CONFIGURACAO_PID = 3

    # Modifique o loop principal para incluir a nova tela
    try:
        pot.counter = 1
        pot.val_max = 2
        while True:
            if dado.telas == dado.TELA_INICIAL:
                lcd.lcd_display_string("**** QUALIFIX **** ", 1, 1)
                lcd.lcd_display_string("Iniciar", 2, 1)
                lcd.lcd_display_string("Configuracoes", 3, 1)
                if pot.counter == 1:
                    lcd.lcd_display_string(">", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                elif pot.counter == 2:
                    lcd.lcd_display_string(">", 3, 0)
                    lcd.lcd_display_string(" ", 2, 0)
                if pot.get_sw_status == 0 and pot.counter == 1:
                    dado.set_telas(dado.TELA_EXECUCAO)
                    pid.set_control_flag(True)
                    lcd.lcd_clear()
                    time.sleep(0.3)
                elif pot.get_sw_status == 0 and pot.counter == 2:
                    pot.counter = 1
                    dado.set_telas(dado.TELA_CONFIGURACAO)
                    lcd.lcd_clear()
                    time.sleep(0.3)

            elif dado.telas == dado.TELA_EXECUCAO:
                lcd.lcd_display_string("Execucao", 1, 1)
                lcd.lcd_display_string(f"1:{pid.value_temp[0]} 2:{pid.value_temp[1]}", 2, 1)
                lcd.lcd_display_string(f"3:{pid.value_temp[2]} 4:{pid.value_temp[3]}", 3, 1)
                lcd.lcd_display_string(f"5:{pid.value_temp[4]} 6:{pid.value_temp[5]}", 4, 1)

                if pot.get_sw_status == 0:
                    dado.set_telas(dado.TELA_INICIAL)
                    pid.set_control_flag(False)
                    lcd.lcd_clear()
                    time.sleep(0.3)

            elif dado.telas == dado.TELA_CONFIGURACAO:
                pot.val_max = 3  # Limita a quantidade de digitos para ajuste de setpoint
                lcd.lcd_display_string("Ajuste setpoint", 1, 1)
                lcd.lcd_display_string(f"Temp. {setpoint}C  ", 2, 1)
                lcd.lcd_display_string("PID: ", 3, 1)
                lcd.lcd_display_string("Sair: ", 4, 1)
                if pot.get_counter() == 1:
                    lcd.lcd_display_string(">", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                    lcd.lcd_display_string(" ", 4, 0)
                    if pot.get_sw_status == 0 and pot.get_counter() == 1:
                        time.sleep(0.6)
                        ajt = 1
                        pot.val_max = 300
                        pot.counter = setpoint
                        while ajt == 1:
                            setpoint = pot.get_counter()
                            lcd.lcd_display_string(f"Temp. {setpoint}C  ", 2, 1)
                            if pot.get_sw_status == 0:
                                pot.val_max = 2
                                pot.counter = pot.val_min

                                save_setpoint_to_file(setpoint)

                                pid.setpoint_list = [setpoint, setpoint, setpoint, setpoint, setpoint, setpoint]
                                ajt = 0
                                pot.val_max = 3
                                pot.counter = 1
                                dado.set_telas(dado.TELA_CONFIGURACAO)
                                lcd.lcd_clear()
                                time.sleep(0.3)

                elif pot.get_counter() == 2:
                    lcd.lcd_display_string(" ", 2, 0)
                    lcd.lcd_display_string(">", 3, 0)
                    lcd.lcd_display_string(" ", 4, 0)
                    if pot.get_sw_status == 0 and pot.get_counter() == 2:
                        dado.set_telas(TELA_CONFIGURACAO_PID)
                        lcd.lcd_clear()
                        time.sleep(0.3)
                elif pot.get_counter() == 3:
                    lcd.lcd_display_string(" ", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                    lcd.lcd_display_string(">", 4, 0)
                    if pot.get_sw_status == 0 and pot.get_counter() == 3:
                        dado.set_telas(dado.TELA_INICIAL)
                        lcd.lcd_clear()
                        time.sleep(0.3)
                        pot.counter = 1
                        pot.val_max = 2

            elif dado.telas == TELA_CONFIGURACAO_PID:
                pot.val_max = 6 # Limita a quantidade de digitos para ajuste de setpoint
                canal = pot.get_counter()
                lcd.lcd_display_string("Ajuste PID", 1, 1)
                lcd.lcd_display_string(f"Canal {canal}", 2, 1)
                lcd.lcd_display_string("Kp: {:.2f} Ki: {:.2f}".format(kp_list[canal-1], ki_list[canal-1]), 3, 1)
                lcd.lcd_display_string("Kd: {:.2f}".format(kd_list[canal-1]), 4, 1)
                # lcd.lcd_display_string("Sair: ", 4, 1)

                if pot.get_sw_status == 0:
                    time.sleep(0.6)
                    pot.val_max = 300 # Limita a quantidade de digitos para ajuste de setpoint
                    ajt = 1
                    lcd.lcd_clear()
                    pot.counter = kp_list[canal-1] * 100
                    while ajt == 1:
                        lcd.lcd_display_string("Ajuste Kp", 1, 1)
                        lcd.lcd_display_string(f"Kp: {kp_list[canal-1]:.2f}", 2, 1)
                        kp_list[canal-1] = pot.get_counter() / 100.0
                        if pot.get_sw_status == 0:
                            time.sleep(0.6)
                            ajt = 2
                            lcd.lcd_clear()
                            pot.counter = ki_list[canal-1] * 100
                            while ajt == 2:
                                lcd.lcd_display_string("Ajuste Ki", 1, 1)
                                lcd.lcd_display_string(f"Ki: {ki_list[canal-1]:.2f}", 2, 1)
                                ki_list[canal-1] = pot.get_counter() / 100.0
                                if pot.get_sw_status == 0:
                                    time.sleep(0.6)
                                    ajt = 3
                                    lcd.lcd_clear()
                                    pot.counter = kd_list[canal-1] * 100
                                    while ajt == 3:
                                        lcd.lcd_display_string("Ajuste Kd", 1, 1)
                                        lcd.lcd_display_string(f"Kd: {kd_list[canal-1]:.2f}", 2, 1)
                                        kd_list[canal-1] = pot.get_counter() / 100.0
                                        if pot.get_sw_status == 0:
                                            ajt = 0
                                            pot.val_max = 3  # Limita a quantidade de digitos para ajuste de setpoint
                                            pot.counter = 1
                                            save_pid_values(kp_list, ki_list, kd_list)  # Salva os valores ajustados
                                            pid.kp_list = kp_list
                                            pid.ki_list = ki_list
                                            pid.kd_list = kd_list
                                            dado.set_telas(dado.TELA_CONFIGURACAO)
                                            lcd.lcd_clear()
                                            time.sleep(0.3)

    except KeyboardInterrupt:
        lcd.lcd_clear()
        pot.cleanup()
        io.io_rpi.cleanup()
        pid.set_control_flag(False)
        pid.stop()
        print("Saindo do programa")
        exit()