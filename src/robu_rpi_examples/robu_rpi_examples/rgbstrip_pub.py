import rclpy
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#ros2 topic pub /color std_msgs/msg/ColorRGBA "{r: 128, g: 0, b: 0, a: 0}" -1
#sudo bash -c 'source install/setup.bash && ros2 run robu rgbstrip'

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rgbstrip')
    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
    pub = node.create_publisher(ColorRGBA, '/color', qos_profile)
    mycolor = ColorRGBA()
    mycolor.r=255.0
    mycolor.g=128.0
    mycolor.b=128.0
    pub.publish(mycolor)
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
