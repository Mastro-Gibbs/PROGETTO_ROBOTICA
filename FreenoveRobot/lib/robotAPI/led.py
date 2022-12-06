import time
from rpi_ws281x import *
from lib.robotAPI.utils import ROBOTAPIConstants as RC


class Led:
    def __init__(self):
        # Control the sending order of color data
        self.ORDER = "RGB"
        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(
            RC.LED_COUNT, RC.LED_PIN, RC.LED_FREQ_HZ, RC.LED_DMA, RC.LED_INVERT, RC.LED_BRIGHTNESS, RC.LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

    def LED_TYPR(self, order, R_G_B):
        B = R_G_B & 255
        G = R_G_B >> 8 & 255
        R = R_G_B >> 16 & 255
        Led_type = ["GRB", "GBR", "RGB", "RBG", "BRG", "BGR"]
        color = [Color(G, R, B), Color(G, B, R), Color(R, G, B),
                 Color(R, B, G), Color(B, R, G), Color(B, G, R)]
        if order in Led_type:
            return color[Led_type.index(order)]

    def colorWipe(self, color, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        color = self.LED_TYPR(self.ORDER, color)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def theaterChase(self, color, wait_ms=50, iterations=10):
        """Movie theater light style chaser animation."""
        color = self.LED_TYPR(self.ORDER, color)
        for j in range(iterations):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, color)
                self.strip.show()
                time.sleep(wait_ms/1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, 0)

    def wheel(self, pos):
        """Generate rainbow colors across 0-255 positions."""
        if pos < 0 or pos > 255:
            r = g = b = 0
        elif pos < 85:
            r = pos * 3
            g = 255 - pos * 3
            b = 0
        elif pos < 170:
            pos -= 85
            r = 255 - pos * 3
            g = 0
            b = pos * 3
        else:
            pos -= 170
            r = 0
            g = pos * 3
            b = 255 - pos * 3
        return self.LED_TYPR(self.ORDER, Color(r, g, b))

    def rainbow(self, wait_ms=20, iterations=1):
        """Draw rainbow that fades across all pixels at once."""
        for j in range(256*iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self.wheel((i+j) & 255))
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def rainbowCycle(self, wait_ms=20, iterations=5):
        """Draw rainbow that uniformly distributes itself across all pixels."""
        for j in range(256*iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self.wheel(
                    (int(i * 256 / self.strip.numPixels()) + j) & 255))
            self.strip.show()
            time.sleep(wait_ms/1000.0)
        self.colorWipe(Color(0, 0, 0), 10)

    def car_arrow(self, clockwise):
        wait_s = 0.5
        data = [0,1,6,7] if clockwise == 0 else [2,3,4,5]

        off    = self.LED_TYPR(self.ORDER, Color(255, 100, 0))
        orange = self.LED_TYPR(self.ORDER, Color(0, 0, 0))

        while True:
            for index in data:
                self.strip.setPixelColor(index, orange)

            self.strip.show()
            time.sleep(wait_s)

            for index in data:
                self.strip.setPixelColor(index, off)

            self.strip.show()
            time.sleep(wait_s)

    def theaterChaseRainbow(self, wait_ms=50):
        """Rainbow movie theater light style chaser animation."""
        for j in range(256):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, self.wheel((i+j) % 255))
                self.strip.show()
                time.sleep(wait_ms/1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, 0)

    def ledIndex(self, index, R, G, B):
        color = self.LED_TYPR(self.ORDER, Color(R, G, B))
        for i in range(8):
            if index & 0x01 == 1:
                self.strip.setPixelColor(i, color)
                self.strip.show()
            index = index >> 1

    def ledMode(self, n):
        self.mode = n
        while True:
            if self.mode == '1':
                self.colorWipe(Color(255, 0, 0))  # Red wipe
                self.colorWipe(Color(0, 255, 0))  # Green wipe
                self.colorWipe(Color(0, 0, 255))  # Blue wipe
                self.colorWipe(Color(0, 0, 0), 10)
            elif self.mode == '2':
                self.theaterChaseRainbow()
                self.colorWipe(Color(0, 0, 0), 10)
            elif self.mode == '3':
                self.rainbow()
                self.colorWipe(Color(0, 0, 0), 10)
            elif self.mode == '4':
                self.rainbowCycle()
                self.colorWipe(Color(0, 0, 0), 10)
            else:
                self.colorWipe(Color(0, 0, 0), 10)
                break
