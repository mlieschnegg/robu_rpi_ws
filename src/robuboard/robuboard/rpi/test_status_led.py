from rpi_ws281x import Adafruit_NeoPixel, Color, ws
import RPi.GPIO as GPIO

GPIO_STATUS_LED = 21

r = 0
g = 255
b = 0
w = 0

status_led = Adafruit_NeoPixel(1, GPIO_STATUS_LED, strip_type=ws.SK6812_STRIP_RGBW)  
status_led.begin()
status_led.setPixelColor(0, Color(r, g, b, w))
status_led.show()
#GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_STATUS_LED, GPIO.IN)
GPIO.cleanup()