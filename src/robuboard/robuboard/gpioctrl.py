import rclpy
from rclpy.node import Node
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.timer
from std_msgs.msg import ByteMultiArray, MultiArrayDimension
from robuboard.rpi.utils import is_raspberry_pi, is_robuboard, is_mmteensy
import robuboard.rpi.robuboard as robuboard
import subprocess
import time
import sys

class PowerSwitch(Node):
    TIMEOUT_POWEROFF_KEY = 2.0                   #seconds
    TIME_POWER_LED_REFRESH = 0.2                       #seconds
    TIMEOUT_POWEROFF_OFF = 20.0                         #seconds
    LED_RGB_POWERON:tuple[int,int,int] = (0, 16, 0)  #(r,g,b)
    LED_RGB_POWEROFF:tuple[int,int,int] = (64, 0, 0) #(r,g,b)

    def __init__(self, node_name : str):
        super().__init__(node_name)

        self._mmt_power_switch_state : list[bool] = [False, False]
        self._cnt_power_led = 0

        if is_raspberry_pi():
            robuboard.init_gpios()
            robuboard.set_status_led(0,0,0)
            self._cnt_power_led = 0
            self._timer_poweron = self.create_timer(
                PowerSwitch.TIME_POWER_LED_REFRESH,
                self._timer_poweron_cb)
        else:
            msg = "This node can only run on a Raspberry Pi!"
            self.get_logger().error(msg)
            self.destroy_node()
            raise Warning(msg)
        

        self._pub_powerswitch_rpi = self.create_publisher(
            ByteMultiArray, 
            "robuboard/rpi/powerswitch_state",
            rclpy.qos.QoSProfile(depth=10))
        
        self._timer_power_switch = self.create_timer(
            1.0, 
            self._timer_power_switch_cb)

        self._sub_powerswtich_mmt = self.create_subscription(ByteMultiArray, 
                                                             "robuboard/mmt/powerswitch_state",
                                                             self._sub_powerswitch_mmt_state_cb,
                                                             rclpy.qos.QoSProfile(depth=10))
    
        self._timer_poweroff : rclpy.timer.Timer | None = None

    def _timer_poweron_cb(self):
        self._cnt_power_led += 1
        r, g, b = PowerSwitch.LED_RGB_POWERON
        cnt_poweroff_max = int(PowerSwitch.TIMEOUT_POWEROFF_KEY / PowerSwitch.TIME_POWER_LED_REFRESH)
        itensity = (self._cnt_power_led / cnt_poweroff_max)

        robuboard.set_status_led(
            int(r * itensity), 
            int(g * itensity),
            int(b * itensity))

        if self._cnt_power_led >= cnt_poweroff_max:
            self._timer_poweron.destroy()
            robuboard.set_status_led(r, g, b)    

    def _timer_power_switch_cb(self):

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

        if robuboard.get_power_switch():
            if self._timer_poweroff == None: #power siwtch is pressed
                self._cnt_power_led = 0
                self._timer_poweroff = self.create_timer(
                    PowerSwitch.TIME_POWER_LED_REFRESH,
                    self._timer_poweroff_cb)
        elif self._timer_poweroff is not None:
            self._timer_poweroff.destroy()
            self._timer_poweroff = None
            r, g, b = PowerSwitch.LED_RGB_POWERON
            robuboard.set_status_led(r,g,b)

        self._pub_powerswitch_rpi.publish(msg)

    def _timer_poweroff_cb(self):
        # robuboard.set_status_led()
        self._cnt_power_led += 1
        
        r, g, b = PowerSwitch.LED_RGB_POWEROFF
        cnt_poweroff_max = int(PowerSwitch.TIMEOUT_POWEROFF_KEY / PowerSwitch.TIME_POWER_LED_REFRESH)
        itensity = (1.0 - self._cnt_power_led / cnt_poweroff_max)
        r = int(r * itensity)
        g = int(g * itensity)
        b = int(b * itensity)

        robuboard.set_status_led(r, g, b)

        if self._cnt_power_led >= cnt_poweroff_max:
            self._timer_poweroff.destroy()
            self._timer_poweroff = None

            #power off device
            self.get_logger().info("Release the power button to shut down the RobuBoard.")
            while robuboard.get_power_switch():
                time.sleep(0.1)

            r, g, b = PowerSwitch.LED_RGB_POWEROFF
            
            shutdown = True
            self.get_logger().info(f"Shutdown initiated ({int(PowerSwitch.TIMEOUT_POWEROFF_OFF):02d} s)")
            for i in range(int(PowerSwitch.TIMEOUT_POWEROFF_OFF), 0, -1):
                if i%2 == 0:
                    robuboard.set_status_led(r, g, b)
                else:
                    robuboard.set_status_led(0, 0, 0)

                sys.stdout.write(f"\rShutdown in {i:02d} s")
                sys.stdout.flush()

                if robuboard.get_power_switch():
                    shutdown = False
                    self.get_logger().info(f"Shutdown has been aborted.")
                    break

                time.sleep(1.0)

            print() #\n
            time.sleep(PowerSwitch.TIMEOUT_POWEROFF_OFF - int(PowerSwitch.TIMEOUT_POWEROFF_OFF))            
            
            if shutdown:
                subprocess.run(["sudo", "shutdown", "now"])
            else:
                r,g,b = PowerSwitch.LED_RGB_POWERON
                robuboard.set_status_led(r, g, b)

    def _sub_powerswitch_mmt_state_cb(self, msg : ByteMultiArray):
        
        #self.get_logger().info(f"teensy power switch state: {msg.data}")
        vals = [bool(val) for val in msg.data]
        if self._mmt_power_switch_state[1] and not vals[1]: #mmty enables the power-ic to switch on
            r, g, b = PowerSwitch.LED_RGB_POWERON
            try:
                if not robuboard.IS_ROBUBOARD_V1:
                    command = f"sudo -E env \"ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY\" \
                        \"RMW_FASTRTPS_USE_SHM=$RMW_FASTRTPS_USE_SHM\" \
                        \"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\" \"RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION\" \
                        \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \
                        \"USER=$USER\" bash -c \
                        'ros2 run robuboard set_status_led --ros-args -p r:={r} -p g:={g} -p b:={b}'"
                    subprocess.run(command, shell=True)
                else:
                    robuboard.set_status_led(r,g,b)
            #   robuboard.disable_5v_supply()
            except:
              pass
        self._mmt_power_switch_state = vals

def main_set_status_led(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("set_status_led")
    mynode.declare_parameters("", [("r", 50), ("g", 10), ("b", 0), ("w", 0)])
    try:
        mynode.get_logger().info("Setting status LED!")
        r = mynode.get_parameter("r").get_parameter_value().integer_value
        g = mynode.get_parameter("g").get_parameter_value().integer_value
        b = mynode.get_parameter("b").get_parameter_value().integer_value
        w = mynode.get_parameter("w").get_parameter_value().integer_value
        mynode.get_logger().info(f"Received LED values: r={r}, g={g}, b={b}, w={w}")
        robuboard.set_status_led(r, g, b, w)
        # rclpy.spin(mynode)
    except KeyboardInterrupt:
        pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_poweroff_robuboard(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("power_off_robuboard")
    try:
        if is_robuboard():
            mynode.get_logger().info("Powering off RobuBoard...")
            robuboard.power_off_robuboard()
        else:
            mynode.get_logger().error("No RobuBoard found!")
    except KeyboardInterrupt:
       pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_reset_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("reset_teensy")
    try:
        if is_robuboard():
            mynode.get_logger().info("Restarting Teensy!")
            robuboard.power_on_teensy()
            robuboard.start_firmware_teensy()
        else:
            mynode.get_logger().error("No RobuBoard found!")
    except KeyboardInterrupt:
       pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_poweroff_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("reset_teensy")
    try:
        if is_robuboard():
            mynode.get_logger().info("Restarting Teensy!")
            robuboard.power_off_teensy()
        else:
            mynode.get_logger().error("No RobuBoard found!")
    except KeyboardInterrupt:
       pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_start_bootloader_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("start_bootloader_teensy")
    mynode.declare_parameter("force", False)
    try:
        if is_mmteensy():
            mynode.get_logger().info("Starting Teensy Bootloader!")
            force = mynode.get_parameter("force").get_parameter_value().bool_value
            robuboard.start_bootloader_teensy(force)
        else:
            mynode.get_logger().error("No Teensy found!")
    except KeyboardInterrupt:
        pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_start_firmware_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("start_firmware_teensy")
    try:
        if is_mmteensy():
            mynode.get_logger().info("Starting Firmware on Teensy!")
            robuboard.start_firmware_teensy()
        else:
            mynode.get_logger().error("No Teensy found!")
    except KeyboardInterrupt:
        pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_upload_firmware_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("upload_firmware_teensy")
    try:
        if is_mmteensy():
            mynode.get_logger().info("Uploading Firmware to Teensy!")
            robuboard.upload_firmware_teensy()
        else:
            mynode.get_logger().error("No Teensy found!")
    except KeyboardInterrupt:
        pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_build_firmware_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("build_firmware_teensy")
    try:
        if is_mmteensy():
            mynode.get_logger().info("Building firmware!")
            robuboard.build_firmware_teensy()
        else:
            mynode.get_logger().error("No Teensy found!")
    except KeyboardInterrupt:
        pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_build_upload_firmware_teensy(args=None):
    rclpy.init(args=args)
    mynode = rclpy.node.Node("build_firmware_teensy")
    try:
        if is_mmteensy():
            mynode.get_logger().info("Building firmware!")
            robuboard.start_bootloader_teensy()
            robuboard.build_firmware_teensy()
            robuboard.upload_firmware_teensy()
        else:
            mynode.get_logger().error("No Teensy found!")
    except KeyboardInterrupt:
        pass
    mynode.destroy_node()
    rclpy.shutdown()

def main_powerswitch(args=None):
    rclpy.init(args=args)
    temp_node = rclpy.create_node('__check_node__')
    node = None
    try:
        nodes_info = temp_node.get_node_names_and_namespaces()
        running_node_names = [name for name, ns in nodes_info]

        target_name = 'power_switch'
        temp_node.destroy_node()

        if target_name not in running_node_names:
            node = PowerSwitch(target_name)
            rclpy.spin(node)

    except KeyboardInterrupt:
        print("Du hast STRG+C gedrückt!")  # STRG+C abfangen
    except Warning as w:
        print(w)
    finally:
        if node is not None:
            node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main_powerswitch()