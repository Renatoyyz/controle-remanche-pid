import time
from Controller.PID import PIDController
from Controller.IOs import IO_MODBUS
from Controller.Dados import Dado
from Controller.Lcd import Lcd
from Controller.KY040 import KY040
import os

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

if __name__ == "__main__":
    setpoint = read_setpoint_from_file()
    kp_list = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    ki_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    kd_list = [ 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
    setpoint_list = [setpoint, setpoint, setpoint, setpoint, setpoint, setpoint]

    dado = Dado()
    lcd = Lcd()
    io = IO_MODBUS( dado=dado)
    pot = KY040( io=io, val_min=1, val_max=2)
    pid = PIDController(setpoint_list=setpoint_list, io_modbus=io, kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, adr=[1, 2, 3, 4, 5, 6])
    pid.start(interval=1)
    

    try:
        while True:
            if dado.telas == dado.TELA_INICIAL:
                lcd.lcd_display_string("*****Qualifix******", 1, 1)
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
                    pid._control_flag = True
                    lcd.lcd_clear()
                    time.sleep(0.3)
                elif pot.get_sw_status == 0 and pot.counter == 2:
                    dado.set_telas(dado.TELA_CONFIGURACAO)
                    lcd.lcd_clear()
                    time.sleep(0.3)

            elif dado.telas == dado.TELA_EXECUCAO:
                lcd.lcd_display_string("Execucao", 1, 1)
                # lcd.lcd_display_string("Pressione o botao", 2, 1)
                lcd.lcd_display_string(f"1:{pid.value_temp[0]} 2:{pid.value_temp[1]}", 2, 1)
                lcd.lcd_display_string(f"3:{pid.value_temp[2]} 4:{pid.value_temp[3]}", 3, 1)
                lcd.lcd_display_string(f"5:{pid.value_temp[4]} 6:{pid.value_temp[5]}", 4, 1)

                if pot.get_sw_status == 0:
                    dado.set_telas(dado.TELA_INICIAL)
                    pid._control_flag = False
                    lcd.lcd_clear()
                    time.sleep(0.3)
            elif dado.telas == dado.TELA_CONFIGURACAO:
                pot.val_max = 2 # Limita a quantidade de digitos para ajusta de setpoint
                lcd.lcd_display_string("Ajuste setpoint", 1, 1)
                lcd.lcd_display_string(f"Temp. {setpoint}C", 2, 1)
                lcd.lcd_display_string("Sair: ", 3, 1)
                if pot.get_counter() == 1:
                    lcd.lcd_display_string(">", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                    lcd.lcd_display_string(" ", 4, 0)
                    if pot.get_sw_status == 0:
                        time.sleep(0.6)
                        ajt = 1
                        pot.val_max = 200
                        pot.counter = setpoint
                        while ajt == 1:
                            setpoint = pot.get_counter()
                            lcd.lcd_display_string(f"Temp. {setpoint}C", 2, 1)
                            if pot.get_sw_status == 0:
                                pot.val_max = 2
                                pot.counter = pot.val_min

                                save_setpoint_to_file(setpoint)

                                pid.setpoint_list = [setpoint, setpoint, setpoint, setpoint, setpoint, setpoint]
                                ajt = 0
                                dado.set_telas(dado.TELA_INICIAL)
                                lcd.lcd_clear()
                                time.sleep(0.3)

                elif pot.get_counter() == 2:
                    lcd.lcd_display_string(" ", 2, 0)
                    lcd.lcd_display_string(">", 3, 0)
                    lcd.lcd_display_string(" ", 4, 0)
                if pot.get_sw_status == 0 and pot.get_counter() == 2:
                    pot.val_max = 2
                    dado.set_telas(dado.TELA_INICIAL)
                    lcd.lcd_clear()
                    time.sleep(0.3)

    except KeyboardInterrupt:
        lcd.lcd_clear()
        pot.cleanup()
        io.io_rpi.cleanup()
        pid._control_flag = False
        pid.stop()
        print("Saindo do programa")
        exit()