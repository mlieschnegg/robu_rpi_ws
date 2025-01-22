
#from uebungen.rpi_utils import is_raspberry_pi
import sys
import signal
import time

import rclpy
from rclpy.node import Node
import rclpy.publisher
import rclpy.qos
import rclpy.timer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, UInt8MultiArray

from robu_rpi_examples.rpi.utils import is_raspberry_pi
from rpi_ws281x import ws, Adafruit_NeoPixel, Color

#https://github.com/rpi-ws281x/rpi-ws281x-python
#https://cdn.sparkfun.com/datasheets/BreakoutBoards/WS2812B.pdf
#Installation der Bibliothek am Raspberry: sudo pip install rpi_ws281x

NAME_DICT = {
    "li":{"katnr": 0, "vorname": "Michael", "color": [255,555,0,0]},
    "ambtin20": {"katnr": 1, "vorname": "Timo", "color": [255,0,0,0]},
    "baylun20": {"katnr": 2, "vorname": "Lukas", "color": [0,255,0,0]},
    "forman21": {"katnr": 3, "vorname": "Marco", "color": [0,0,255,0]},
    "dorjon21": {"katnr": 4, "vorname": "Jonas", "color": [255,0,0,0]},
    "engsan21": {"katnr": 5, "vorname": "Samuel", "color": [0,255,0,0]},
    "fasmin21": {"katnr": 6, "vorname": "Michael", "color": [0,0,255,0]},
    "ferdon21": {"katnr": 7, "vorname": "Dominik", "color": [255,0,0,0]},
    "finman21": {"katnr": 8, "vorname": "Maximilian", "color": [0,255,0,0]},
    "frisan21": {"katnr": 9, "vorname": "Sarah", "color": [0,0,255,0]},
    "hadman21": {"katnr": 10,"vorname": "Matthias", "color": [255,0,0,0]},
    "hauran21": {"katnr": 11,"vorname": "Rafael", "color": [0,255,0,0]},
    "husfln21": {"katnr": 12,"vorname": "Florentina", "color": [0,0,255,0]},
    "hutaln21": {"katnr": 13,"vorname": "Alexander", "color": [255,0,0,0]},
    "kiealn20": {"katnr": 14,"vorname": "Alexander", "color": [0,255,0,0]},
    "kocman21": {"katnr": 15,"vorname": "Maximilian", "color": [0,0,255,0]},
    "krealn21": {"katnr": 16,"vorname": "Alexander", "color": [255,0,0,0]},
    "magman21": {"katnr": 17,"vorname": "Matthias", "color": [0,255,0,0]},
    "orophn21": {"katnr": 18,"vorname": "Philip", "color": [0,0,255,0]},
    "seuchn20": {"katnr": 19,"vorname": "Christoph", "color": [255,0,0,0]},
    "soelun21": {"katnr": 20,"vorname": "Lukas", "color": [0,255,0,0]},
    "swenin21": {"katnr": 21,"vorname": "Nico", "color": [0,0,255,0]},
    "taxpan20": {"katnr": 22,"vorname": "Patrick", "color": [255,0,0,0]},
    "weijun21": {"katnr": 23,"vorname": "Julian", "color": [0,255,0,0]}
}

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

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        #Format: [lednr1 r1, g1, b1, w1, lednr2, r2, g2, b2, w2]
        self.create_subscription(UInt8MultiArray, "team1", self._team1_callback, qos_profile)
        self.create_subscription(UInt8MultiArray, "team2", self._team2_callback, qos_profile)
        self.create_subscription(UInt8MultiArray, "team3", self._team3_callback, qos_profile)
        self.create_subscription(UInt8MultiArray, "team4", self._team4_callback, qos_profile)
        self.create_subscription(UInt8MultiArray, "team5", self._team5_callback, qos_profile)
        self.create_subscription(UInt8MultiArray, "team6", self._team6_callback, qos_profile)
        self.create_subscription(String, "user", self._user_callback, 10)

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


    def _team1_callback(self, msg:UInt8MultiArray):
        #msg.data Format: lednr0, red0, green0, blue0, white0, lednr1, red1, green1, blue1, white1, ...
        led_vals = list(msg.data)
        self._update_team(led_vals, 0)
        
    def _team2_callback(self, msg:UInt8MultiArray):
        led_vals = list(msg.data)
        self._update_team(led_vals, 1)

    def _team3_callback(self, msg:UInt8MultiArray):
        led_vals = list(msg.data)
        self._update_team(led_vals, 2)

    def _team4_callback(self, msg:UInt8MultiArray):
        led_vals = list(msg.data)
        self._update_team(led_vals, 3)

    def _team5_callback(self, msg:UInt8MultiArray):
        led_vals = list(msg.data)
        self._update_team(led_vals, 4)

    def _team6_callback(self, msg:UInt8MultiArray):
        led_vals = list(msg.data)
        self._update_team(led_vals, 5)

    def _user_callback(self, msg:String):
        #Format = "benutzername" e.q li
        try:
            
            lednr = NAME_DICT[msg.data]["katnr"]
            color = NAME_DICT[msg.data]["color"]
            if max(self._led_values[lednr]):
                self._led_values[lednr] = [0, 0, 0, 0]
                self.get_logger().info(f"{NAME_DICT[msg.data]['vorname']} -> LED wurde DEAKTIVIERT!")
            else:
                self._led_values[lednr] = color
                self.get_logger().info(f"{NAME_DICT[msg.data]['vorname']} -> LED wurde AKTIVIERT!")
            self._hw_led_strip_set_color()
        except Exception as warn:
            self.get_logger().warn(f"{warn}")
    

    def _update_team(self, led_vals:list, teamnr:int=0):
        #Absicherung falls der Benutzer eine falsche Länge übergibt
        if (len(led_vals) % self.TEAM_LED_NUM == 0 ): 
            for i in range(0, len(led_vals), self.TEAM_LED_NUM):
                led_nr, red_val, green_val, blue_val, white_val = led_vals[i:i+self.TEAM_LED_NUM]
                #Absicherung -> jedes Team hat nur self.TEAM_LED_NUM LEDs
                if led_nr < self.TEAM_LED_NUM:
                    self._led_values[self.TEAM_LED_OFFSET + teamnr*self.TEAM_LED_NUM+led_nr] = [red_val, green_val, blue_val, white_val]
                    self.get_logger().info(f"Farbwert von Team {teamnr+1}/LED {led_nr} -> {[red_val, green_val, blue_val, white_val]}")
            self._hw_led_strip_set_color()

    def _hw_led_strip_begin(self):
        if self._is_rpi:
            self._hw_led_values.begin()
            self._hw_led_strip_set_color()

    def _hw_led_strip_set_color(self):
        if self._is_rpi:
            for i, (r,g,b,w) in enumerate(self._led_values):
                self._hw_led_values.setPixelColorRGB(i, g, r, b, w)
            self._hw_led_values.show()

    def __del__(self):
        if self._is_rpi:
            for i in range(len(self._led_values)):
                self._led_values[i] = [0, 0, 0, 0]
            self._hw_led_strip_set_color()

def main(args=None):
    global node
    rclpy.init(args=args)
    node = LEDDisplay('plf01_ledstrip')

    node.get_logger().info("plf01_ledstrip started!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Du hast STRG+C gedrückt!")  # STRG+C abfangen
    finally:
        node.get_logger().info(f"Node {node.get_name()} wird beendet!")
        node.destroy_node()
        del node
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
