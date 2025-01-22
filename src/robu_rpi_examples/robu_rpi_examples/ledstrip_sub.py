
#from uebungen.rpi_utils import is_raspberry_pi
import sys
import signal

import rclpy
from rclpy.node import Node
import rclpy.publisher
import rclpy.qos
import rclpy.timer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String, ByteMultiArray, MultiArrayDimension, Int8MultiArray

from robu_rpi_examples.rpi.utils import is_raspberry_pi
from rpi_ws281x import ws, Adafruit_NeoPixel, Color

import rclpy

#https://github.com/rpi-ws281x/rpi-ws281x-python
#https://cdn.sparkfun.com/datasheets/BreakoutBoards/WS2812B.pdf
#Installation der Bibliothek am Raspberry: sudo pip install rpi_ws281x

#ros2 service call /camera_bl leddisplay_interfaces/srv/LEDCamera "{camera: left, brightness_percent: 50}"

def new_ledstrip(numleds:int) -> Adafruit_NeoPixel:
    if not is_raspberry_pi():
        return None
    
    LED_PIN = 21            #=GPIO21 am Raspberry: 
                            #Dieser Pin verfügt über PWM-Peripherie und PCM-Peripherie
    LED_FREUQ_HZ = 800000   #Kommunikationsfrequenz 800kHz -> T=1.25 µs
    LED_DMA = 10            #Direct Memory Access (=Hardware-Peripherie)
    LED_BRIGHTNESS = 255    #Höchster Wert bei 8-Bit = 2^8 = 256 -> 0 bis 255

    LED_INVERT = False      #LEDs sind direkt mit dem Raspberry verbunden (kein Transistor)
    LED_CHANNEL = 0         #0 or 1 -> 0 für PWM
    LED_STRIP = ws.SK6812_STRIP_RGBW 

    return  Adafruit_NeoPixel(numleds,
                              LED_PIN,
                              LED_FREUQ_HZ,
                              LED_DMA,
                              LED_INVERT,
                              LED_BRIGHTNESS,
                              LED_CHANNEL,
                              LED_STRIP)

class LEDDisplay(Node):

    TEAM_LED_OFFSET:int = 30
    TEAM_LED_NUM:int = 5

    def __init__(self, node_name: str):
        super().__init__(node_name)

        #Format: [lednr1 r1, g1, b1, w1, lednr2, r2, g2, b2, w2]
        self.create_subscription(Int8MultiArray, "team1", self._team1_callback, 10)

        self._team_leds_config = [
            {"pos": list(range(self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*0, self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*1)), "default-color": [0,0,0,255]},
            {"pos": list(range(self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*1, self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*2)), "default-color": [0,0,0,255]},
            {"pos": list(range(self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*2, self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*3)), "default-color": [0,0,0,255]},
            {"pos": list(range(self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*3, self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*4)), "default-color": [0,0,0,255]},
            {"pos": list(range(self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*4, self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*5)), "default-color": [0,0,0,255]},
            {"pos": list(range(self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*5, self.TEAM_LED_OFFSET+self.TEAM_LED_NUM*6)), "default-color": [0,0,0,255]},
        ]
        self._user_led_config = {"pos": list(range(0, self.TEAM_LED_OFFSET)), "default-color": [255,128,0,0]}


        numleds = max(
            [ max(config["pos"]) for config in self._team_leds_config ] + 
            self._user_led_config["pos"]) + 1
        
        #pos = range(min(pos), max(pos)+1)        
        self._led_values = numleds*[[0,0,0,0]]
        for config in self._team_leds_config:
            for pos in config["pos"]:
                self._led_values[pos] = config["default-color"]

        for pos in self._user_led_config["pos"]:
            self._led_values[pos] = self._user_led_config["default-color"]

        self._is_rpi = is_raspberry_pi()
        self._hw_led_values = new_ledstrip(numleds)
        self._hw_led_strip_begin()


    def _team1_callback(self, msg:Int8MultiArray):
        #length: lednr, red, green, blue, white
        DATA_LEN = 5
        for i in range(0, len(msg.data), DATA_LEN):
            led_nr, red_val, green_val, blue_val, white_val = msg.data[i:i+DATA_LEN]
            self._led_values[led_nr] = [red_val, green_val, blue_val, white_val]

        self._hw_led_strip_set_color()

    def _hw_led_strip_begin(self):
        if self._is_rpi:
            self._hw_led_values.begin()
            self._hw_led_strip_set_color()

    def _hw_led_strip_set_color(self):
        if self._is_rpi:
            for i, (r,g,b,w) in enumerate(self._led_values):
                self._hw_led_values.setPixelColorRGB(i,r, g, b, w)
            self._hw_led_values.show()

    def __del__(self):
        if self._is_rpi:
            for i in range(self._hw_led_values.numPixels()):
                self._hw_led_values.setPixelColorRGB(i,0, 0, 0, 0)

def main(args=None):
    global node
    rclpy.init(args=args)
    node = LEDDisplay('plf01_ledstrip')

    node.get_logger().info("plf01_dedstrip started!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Du hast STRG+C gedrückt!")  # STRG+C abfangen
    finally:
        node.get_logger().info(f"Node {node.get_name()} wird beendet!")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
