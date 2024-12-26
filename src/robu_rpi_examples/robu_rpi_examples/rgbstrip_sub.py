import rclpy
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rpi_ws281x import Adafruit_NeoPixel, Color, ws

#ros2 topic pub /color std_msgs/msg/ColorRGBA "{r: 128, g: 0, b: 0, a: 0}" -1
#sudo bash -c 'source install/setup.bash && ros2 run robu rgbstrip'

# Define the number of WS2812B LED lights and the pin they are connected to
LED_COUNT = 3  # Number of LED
LED_PIN = 21  # 0- number will form a color
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal (Between 1 and 14)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0       # 0 or 1
LED_STRIP = ws.WS2811_STRIP_GRB

# Initialize the WS2812B LED lights
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ,
                        LED_DMA, LED_INVERT, LED_BRIGHTNESS,
                        LED_CHANNEL, LED_STRIP)  
strip.begin()
strip.setPixelColor(0, Color(255, 0, 0))
strip.setPixelColor(1, Color(0, 255, 0))
strip.setPixelColor(2, Color(0, 0, 255))
strip.show()

# Callback function for the color message
def color_callback(msg):
    global node
    node.get_logger().info(f"new color: {msg.r}, {msg.g}, {msg.b}")
    for i in range(LED_COUNT):
        strip.setPixelColor(i, Color(int(msg.r), int(msg.g), int(msg.b)))
    strip.show()

def main(args=None):
    global node
    rclpy.init(args=args)
    node = rclpy.create_node('rgbstrip')
    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    subscription = node.create_subscription(ColorRGBA, 'color', color_callback, qos_profile)
    subscription # prevent unused variable warning
    node.get_logger().info("rgbstrip started!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Du hast STRG+C gedrückt!")  # STRG+C abfangen
    finally:
        # mycolor = ColorRGBA()
        # mycolor.r=0.0
        # mycolor.g=0.0
        # mycolor.b=0.0
        color_callback(ColorRGBA())
        node.get_logger().info(f"Node {node.get_name()} wird beendet!")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
