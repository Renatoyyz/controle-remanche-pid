import time

class KY040:
    def __init__(self, clk_pin=23, dt_pin=22, sw_pin=4, io=None, val_min=1, val_max=10):
        self.clk_pin = clk_pin
        self.dt_pin = dt_pin
        self.sw_pin = sw_pin
        self.io = io
        self.val_min = val_min
        self.val_max = val_max

        self.counter = 0
        self.test_counter = 0
        self.sw_status = False

        self.io.setup(self.clk_pin, self.io.IN, pull_up_down=self.io.PUD_UP)
        self.io.setup(self.dt_pin, self.io.IN, pull_up_down=self.io.PUD_UP)
        self.io.setup(self.sw_pin, self.io.IN, pull_up_down=self.io.PUD_UP)

        self.last_clk_state = self.io.input(self.clk_pin)

        self.io.add_event_detect(self.clk_pin, edge=self.io.BOTH, callback=self._clk_callback, bouncetime=1)
        self.io.add_event_detect(self.sw_pin, self.io.BOTH, callback=self._sw_callback, bouncetime=100)

    def _clk_callback(self, channel):
        clk_state = self.io.input(self.clk_pin)
        dt_state = self.io.input(self.dt_pin)
        if clk_state != self.last_clk_state:
            if dt_state != clk_state:
                self.test_counter += 1
                if self.test_counter % 2 == 0:
                    self.counter += 1
                if self.counter > self.val_max:
                    self.counter = self.val_min
            else:
                self.test_counter -= 1
                if self.test_counter % 2 == 0:
                    self.counter -= 1

                if self.counter < self.val_min:
                    self.counter = self.val_max

            self.last_clk_state = clk_state

    def _sw_callback(self, channel):
        self.sw_status = self.io.input(self.sw_pin)

    def get_counter(self):
        return self.counter

    def get_sw_status(self):
        return self.sw_status

    def cleanup(self):
        self.io.remove_event_detect(self.clk_pin)
        self.io.remove_event_detect(self.sw_pin)
        self.io.cleanup()

# Exemplo de uso
if __name__ == "__main__":
    from IOs import IO_MODBUS
    ios = IO_MODBUS()
    ky040 = KY040(io=ios.io_rpi.GPIO)
    try:
        while True:
            print("Counter: ", ky040.get_counter())
            print("SW Status: ", ky040.get_sw_status())
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
        ky040.cleanup()