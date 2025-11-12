from rclpy.node import Node
import rclpy

import os
import time

from smbus import SMBus

from ssd1306.common.ssd1306 import SSD1306
from robuboard.rpi.utils import is_raspberry_pi
from robuboard.rpi.netinfo import get_current_wifi_signal, get_interface, get_ip_address, get_ssid

def init_display():
    """Initialisiert das OLED-Display."""
    i2cbus = SMBus(1)
    return SSD1306(i2cbus)

def draw_info_network(oled:SSD1306, hostname:str, ip:str, ssid:str, signal:int):
    """Zeichnet die Infos auf dem Display."""
    oled.canvas.rectangle((0, 0, oled.width-1, oled.height-1), outline=1, fill=0)

    oled.canvas.text((5, 5),  f"HN: {hostname}", fill=1)
    oled.canvas.text((5, 20), f"IP: {ip}", fill=1)
    oled.canvas.text((5, 35), f"SSID: {ssid}", fill=1)
    oled.canvas.text((5, 50), f"Signal: {signal} dbm", fill=1)

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
        self._rid:int = -1 #ROS_DOMAIN_ID

        self._oled = init_display()
        self._timer_display_info = self.create_timer(10.0, self._timer_display_info_cb)

        self._timer_display_info_cb()

    def _timer_display_info_cb(self):
        hostname = os.uname().nodename
        ip_address = get_ip_address()
        ssid, signal = get_current_wifi_signal()
        ros_domain_id = os.getenv("ROS_DOMAIN_ID")
        self._oled.bus.close()
        if self._hostname != hostname or self._ip_address != ip_address or self._ssid != ssid:
            self._hostname = hostname
            self._ip_address = ip_address
            self._ssid = ssid
            draw_info_network(self._oled, hostname, ip_address, ssid, signal)

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    if not is_raspberry_pi():
        print("Dieser Node ist nur auf einem Raspberry ausführbar!")
        exit()
    try:
        rclpy.init()
        try:
            node = ConnectionDisplayNode("connection_display")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

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


