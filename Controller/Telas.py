import time
from luma.core.interface.serial import spi
from luma.oled.device import sh1106
from luma.core.render import canvas

class ST7920Display:
    def __init__(self, port=0, device=0, gpio_RST=25):
        self.serial = spi(port=port, device=device, gpio_RST=gpio_RST)
        self.device = sh1106(self.serial, width=128, height=64)

    def clear(self):
        self.device.clear()

    def display_text(self, text, x=0, y=0):
        with canvas(self.device) as draw:
            draw.text((x, y), text, fill="white")

    def display_image(self, image):
        self.device.display(image)

    def display_line(self, x1, y1, x2, y2):
        with canvas(self.device) as draw:
            draw.line((x1, y1, x2, y2), fill="white")

    def display_rectangle(self, x1, y1, x2, y2):
        with canvas(self.device) as draw:
            draw.rectangle((x1, y1, x2, y2), outline="white", fill="black")

    def display_circle(self, x, y, r):
        with canvas(self.device) as draw:
            draw.ellipse((x-r, y-r, x+r, y+r), outline="white", fill="black")

# Exemplo de uso
if __name__ == "__main__":
    display = ST7920Display(gpio_RST=25)  # Especifique o pino GPIO para RST
    display.clear()
    display.display_text("Hello, World!", 10, 10)
    time.sleep(5)
    display.clear()