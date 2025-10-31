# i2c bus (0 -- original Pi, 1 -- Rev 2 Pi)
I2CBUS = 1

import smbus
from time import sleep, strftime
import subprocess
import threading

class i2c_device:
   def __init__(self, addr, port=I2CBUS, timeout=1):
      self.addr = addr
      self.bus = smbus.SMBus(port)
      self.timeout = timeout
      self.lock = threading.Lock()  # Lock para evitar race conditions

# Write a single command
   def write_cmd(self, cmd):
      with self.lock:
         try:
            self.bus.write_byte(self.addr, cmd)
            sleep(0.0001)
         except Exception as e:
            print(f"Erro ao escrever comando I2C: {e}")

# Write a command and argument
   def write_cmd_arg(self, cmd, data):
      with self.lock:
         try:
            self.bus.write_byte_data(self.addr, cmd, data)
            sleep(0.0001)
         except Exception as e:
            print(f"Erro ao escrever comando com argumento I2C: {e}")

# Write a block of data
   def write_block_data(self, cmd, data):
      with self.lock:
         try:
            self.bus.write_block_data(self.addr, cmd, data)
            sleep(0.0001)
         except Exception as e:
            print(f"Erro ao escrever bloco de dados I2C: {e}")

# Read a single byte
   def read(self):
      with self.lock:
         try:
            return self.bus.read_byte(self.addr)
         except Exception as e:
            print(f"Erro ao ler byte I2C: {e}")
            return None

# Read
   def read_data(self, cmd):
      with self.lock:
         try:
            return self.bus.read_byte_data(self.addr, cmd)
         except Exception as e:
            print(f"Erro ao ler dado I2C: {e}")
            return None

# Read a block of data
   def read_block_data(self, cmd):
      with self.lock:
         try:
            return self.bus.read_block_data(self.addr, cmd)
         except Exception as e:
            print(f"Erro ao ler bloco de dados I2C: {e}")
            return None


# commands
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80

# flags for display entry mode
LCD_ENTRYRIGHT = 0x00
LCD_ENTRYLEFT = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

# flags for display on/off control
LCD_DISPLAYON = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON = 0x02
LCD_CURSOROFF = 0x00
LCD_BLINKON = 0x01
LCD_BLINKOFF = 0x00

# flags for display/cursor shift
LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE = 0x00
LCD_MOVERIGHT = 0x04
LCD_MOVELEFT = 0x00

# flags for function set
LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE = 0x08
LCD_1LINE = 0x00
LCD_5x10DOTS = 0x04
LCD_5x8DOTS = 0x00

# flags for backlight control
LCD_BACKLIGHT = 0x08
LCD_NOBACKLIGHT = 0x00

En = 0b00000100 # Enable bit
Rw = 0b00000010 # Read/Write bit
Rs = 0b00000001 # Register select bit

class Lcd:
   #initializes objects and lcd
    def __init__(self):
        self.addresses = self.find_i2c_address()
        if self.addresses == []:
            raise Exception(f"LCD not found at address. Available addresses: {self.addresses}")
        self.lcd_device = i2c_device(self.addresses[0])

        self.lcd_write(0x03)
        self.lcd_write(0x03)
        self.lcd_write(0x03)
        self.lcd_write(0x02)

        self.lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE)
        self.lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON)
        self.lcd_write(LCD_CLEARDISPLAY)
        self.lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT)
        sleep(0.2)

    def find_i2c_address(self):
        """Encontra endereço I2C com timeout"""
        try:
            # Adiciona timeout ao subprocess
            output = subprocess.check_output(
                ['i2cdetect', '-y', str(I2CBUS)],
                timeout=5  # ✅ IMPORTANTE: Timeout de 5 segundos
            )
            output = output.decode('utf-8').split('\n')
            addresses = []
            for line in output:
                if line.startswith(' '):
                    continue
                parts = line.split()
                for part in parts[1:]:
                    if part != '--':
                        try:
                            addresses.append(int(part, 16))
                        except ValueError:
                            continue
            return addresses
        except subprocess.TimeoutExpired:
            print("Timeout ao detectar endereço I2C. Usando fallback.")
            return [0x27]  # Endereço padrão comum para LCD I2C
        except Exception as e:
            print(f"Erro ao encontrar endereço I2C: {e}")
            return [0x27]  # Fallback


    # clocks EN to latch command
    def lcd_strobe(self, data):
        try:
            self.lcd_device.write_cmd(data | En | LCD_BACKLIGHT)
            sleep(.0005)
            self.lcd_device.write_cmd(((data & ~En) | LCD_BACKLIGHT))
            sleep(.0001)
        except Exception as e:
            print(f"Erro ao fazer strobe: {e}")

    def lcd_write_four_bits(self, data):
        try:
            self.lcd_device.write_cmd(data | LCD_BACKLIGHT)
            self.lcd_strobe(data)
        except Exception as e:
            print(f"Erro ao escrever 4 bits: {e}")

    # write a command to lcd
    def lcd_write(self, cmd, mode=0):
        try:
            self.lcd_write_four_bits(mode | (cmd & 0xF0))
            self.lcd_write_four_bits(mode | ((cmd << 4) & 0xF0))
        except Exception as e:
            print(f"Erro ao escrever comando: {e}")

    # write a character to lcd (or character rom) 0x09: backlight | RS=DR<
    # works!
    def lcd_write_char(self, charvalue, mode=1):
        try:
            self.lcd_write_four_bits(mode | (charvalue & 0xF0))
            self.lcd_write_four_bits(mode | ((charvalue << 4) & 0xF0))
        except Exception as e:
            print(f"Erro ao escrever caractere: {e}")
  
    # put string function with optional char positioning
    def lcd_display_string(self, string, line=1, pos=0):
        try:
            if line == 1:
                pos_new = pos
            elif line == 2:
                pos_new = 0x40 + pos
            elif line == 3:
                pos_new = 0x14 + pos
            elif line == 4:
                pos_new = 0x54 + pos

            self.lcd_write(0x80 + pos_new)

            for char in string:
                self.lcd_write(ord(char), Rs)
        except Exception as e:
            print(f"Erro ao exibir string no LCD: {e}")

    # clear lcd and set to home
    def lcd_clear(self):
        try:
            self.lcd_write(LCD_CLEARDISPLAY)
            self.lcd_write(LCD_RETURNHOME)
        except Exception as e:
            print(f"Erro ao limpar LCD: {e}")

    # define backlight on/off (lcd.backlight(1); off= lcd.backlight(0)
    def backlight(self, state): # for state, 1 = on, 0 = off
        try:
            if state == 1:
                self.lcd_device.write_cmd(LCD_BACKLIGHT)
            elif state == 0:
                self.lcd_device.write_cmd(LCD_NOBACKLIGHT)
        except Exception as e:
            print(f"Erro ao controlar backlight: {e}")

    # add custom characters (0 - 7)
    def lcd_load_custom_chars(self, fontdata):
        try:
            self.lcd_write(0x40)
            for char in fontdata:
                for line in char:
                    self.lcd_write_char(line)
        except Exception as e:
            print(f"Erro ao carregar caracteres customizados: {e}")

    # put string function with inverted background
    def lcd_display_string_inverter(self, string, line=1, pos=0):
        try:
            if line == 1:
                pos_new = pos
            elif line == 2:
                pos_new = 0x40 + pos
            elif line == 3:
                pos_new = 0x14 + pos
            elif line == 4:
                pos_new = 0x54 + pos

            self.lcd_write(0x80 + pos_new)

            # Turn off backlight to simulate inverted background
            self.backlight(0)
            for char in string:
                self.lcd_write(ord(char), Rs)
            # Turn on backlight after writing the string
            self.backlight(1)
        except Exception as e:
            print(f"Erro ao exibir string invertida: {e}")

        

if __name__ == "__main__":
    lcdi2c = Lcd()

    #Exibe informacoes iniciais
    lcdi2c.lcd_display_string("Renato Oliveira", 1,1)
    # lcdi2c.lcd_display_string("Katia Amor", 2,1)
    # Mostra uma string com fundo invertido
    lcdi2c.lcd_display_string_inverter("Katia Amor", 2, 1)
    sleep(4)

    #Apaga o display
    # lcdi2c.lcd_clear()
    try:
        while True:
            #Mostra a data no display
            lcdi2c.lcd_display_string("Data: %s" %strftime("%d/%m/%y"), 3,1)
            lcdi2c.lcd_display_string("Hora: %s" %strftime("%H/%M/%S"), 4,1)
            sleep(1) 
    except KeyboardInterrupt:
        lcdi2c.lcd_clear()
        print("Fim do programa")