import rclpy
from rclpy.node import Node
import rclpy.publisher
import rclpy.qos
import rclpy.timer
from std_msgs.msg import Bool, String, ByteMultiArray, MultiArrayDimension
from robuboard.rpi.utils import is_raspberry_pi
import robuboard.rpi.robuboard as robuboard

class PowerSwitch(Node):
    _pub_powerswitch_rpi : rclpy.publisher.Publisher = None
    _timer_power_switch : rclpy.timer.Timer = None
    _mmt_power_switch_state : list[bool] = []

    def __init__(self, node_name : str):
        super().__init__(node_name)
        if is_raspberry_pi():
            robuboard.init_gpios()

        self._pub_powerswitch_rpi = self.create_publisher(ByteMultiArray, 
                                                          "robuboard/rpi/powerswitch_state", 
                                                          rclpy.qos.QoSProfile(depth=10))
        
        self._timer_power_switch = self.create_timer(1.0, self._timer_power_switch_callback)

        self._sub_powerswtich_mmt = self.create_subscription(ByteMultiArray, 
                                                             "robuboard/mmt/powerswitch_state",
                                                             self._sub_powerswitch_mmt_state,
                                                             rclpy.qos.QoSProfile(depth=10))
    
    def _timer_power_switch_callback(self):

        # Boolean-Werte in Bytes umwandeln
        try:
            bool_values = [robuboard.get_power_switch(), robuboard.is_on_5v_supply()]
        except:
            bool_values = [False, False]
        
        byte_values = [b'\x01' if value else b'\x00' for value in bool_values]
        #byte_values = bytearray([int(val) for val in bool_values])  # True -> 1, False -> 0

        # MultiArray vorbereiten
        msg = ByteMultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='booleans', size=len(byte_values), stride=1))
        msg.data = byte_values

        self._pub_powerswitch_rpi.publish(msg)

    def _sub_powerswitch_mmt_state(self, msg : ByteMultiArray):
        
        #self.get_logger().info(f"teensy power switch state: {msg.data}")
        self._mmt_power_switch_state = [bool(val) for val in msg.data]

        # if self._mmt_power_switch_state[1]: #mmty enables the power-ic to switch on
        #     try:
        #       robuboard.disable_5v_supply()
        #     except:
        #       pass
        
           

def main_reset_teensy(args=None):
    rclpy.init(args=args)
    try:
        print("Restarting Teensy!")
        robuboard.power_on_teensy()
    except:
       pass
    rclpy.shutdown()

def main_powerswitch(args=None):
  rclpy.init(args=args)
  node = PowerSwitch("PowerSwitch")
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
    main_powerswitch()