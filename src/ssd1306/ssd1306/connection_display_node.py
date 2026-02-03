from rclpy.node import Node
import rclpy

import os
import time

from smbus3 import SMBus

from robuboard.rpi.utils import i2c_ping

from ssd1306.common.ssd1306 import SSD1306
from robuboard.rpi.utils import is_raspberry_pi
from robuboard.rpi.netinfo import get_current_wifi_signal, get_interface, get_ip_address, get_ssid



def init_display():
    """Initialisiert das OLED-Display."""
    ack = i2c_ping(0, 0x3D, 1)
    print("ACK: ", ack)
    if i2c_ping(0, 0x3D, 1) == True:
         i2cbus = SMBus(0)
         return SSD1306(i2cbus, address=0x3D)
    else:
        i2cbus = SMBus(1)
        return SSD1306(i2cbus, adress=0x3C)
    

def draw_info_network(oled:SSD1306, hostname:str, ip:str, ssid:str, signal:int, rid:int=0):
    """Zeichnet die Infos auf dem Display."""
    oled.canvas.rectangle((0, 0, oled.width-1, oled.height-1), outline=1, fill=0)

    oled.canvas.text((5, 5),  f"HN: {hostname}", fill=1)
    oled.canvas.text((5, 20), f"IP: {ip}", fill=1)
    oled.canvas.text((5, 35), f"SSID: {ssid}", fill=1)
    oled.canvas.text((5, 50), f"SIG: {signal} dBm,ROSID: {rid}", fill=1)

    oled.display()

def draw_info_ros(oled:SSD1306, user="robu", pw="robu", id:int=30):
    """Zeichnet die Infos auf dem Display."""
    oled.canvas.rectangle((0, 0, oled.width-1, oled.height-1), outline=1, fill=0)

    oled.canvas.text((5, 5),  f"Linux USER: {user}", fill=1)
    oled.canvas.text((5, 20), f"PW: {user}", fill=1)
    oled.canvas.text((5, 35), f"ROS_DOMAIN_ID: {id}", fill=1)

    oled.display()

class ConnectionDisplayNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self._hostname:str = ""
        self._ip_address:str = ""
        self._ssid:str = ""
        self._signal:int = 0
        self._rid:int = -1 #ROS_DOMAIN_ID
        self._bus = 0

        self._initialized:bool = False

        self._display_init()
        self.display_cls()

        self._timer_display_info = self.create_timer(10.0, self._timer_display_info_cb)

        self._timer_display_info_cb()

    def _display_init(self):
        if not self._initialized:

            ack = i2c_ping(0, 0x3D, 1)
            print("ACK: ", ack)
            if ack == True:
                self._bus = 0
                self._oled:SSD1306 = SSD1306(SMBus(self._bus), address=0x3D)
            else:
                self._bus = 1
                self._oled:SSD1306 = SSD1306(SMBus(self._bus), address=0x3C)

            self._oled.set_on(True)
            self._oled.bus.close()
            self._initialized = True
        return self._initialized

    def display_cls(self):
        if not self._initialized:
            return
        self._oled.bus.open(self._bus)
        self._oled.cls()
        self._oled.bus.close()

    def display_connection_info(self):
        if not self._initialized:
            return
        self._oled.bus.open(self._bus)
        self._oled.cls()
        draw_info_network(self._oled, self._hostname, self._ip_address, self._ssid, self._signal)
        self._oled.bus.close()  

    def update_connection_info(self):
        if not self._initialized:
            return
        hostname = os.uname().nodename
        ip_address = get_ip_address()
        ssid, signal = get_current_wifi_signal()
        ros_domain_id = int(os.getenv("ROS_DOMAIN_ID", 0))
        if self._hostname != hostname or \
            self._ip_address != ip_address or \
            self._ssid != ssid or \
            self._rid != ros_domain_id:

            self._hostname = hostname
            self._ip_address = ip_address
            self._ssid = ssid
            self._signal = signal
            self._rid = ros_domain_id
            self.display_connection_info()

    def _timer_display_info_cb(self):
        self.get_logger().debug("Update des OLED-Displays")
        self._display_init()
        self.update_connection_info()


    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    rclpy.init()
    node = ConnectionDisplayNode("connection_display")
    if not is_raspberry_pi():
        print("Dieser Node ist nur auf einem Raspberry ausführbar!")
        exit()
    try:
        # rclpy.init()
        # try:
        #     node = ConnectionDisplayNode("connection_display")
        # except Exception as e:
        #     print(f"Fehler beim Erstellen des Nodes: {e}")
        #     return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Sie haben STRG+C gedrückt!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# =======================
#   Hauptprogramm
# =======================

if __name__ == "__main__":
    oled = init_display()

    hostname = os.uname().nodename
    ip_address = get_ip_address()
    # interface = get_interface()
    # ssid = get_ssid(interface)
    ssid, signal = get_current_wifi_signal()
    
    ros_domain_id = os.getenv("ROS_DOMAIN_ID")

    draw_info_network(oled, hostname, ip_address, ssid,signal)
    # time.sleep(1)
    # draw_info_ros(oled, id=ros_domain_id)


