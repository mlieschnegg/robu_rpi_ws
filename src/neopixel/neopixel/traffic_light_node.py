from rclpy.node import Node
import rclpy

from neopixel.ledlib import neopixel_spidev as np
from enum import Enum

#Koordinator-Ampel
class TrafficLightMode(Enum):
    STANDARD = 0
    STANDARD_RY = 1
    MANUAL_RED = 2
    MANUAL_GREEN = 3
    MANUAL_YELLOW = 4
    BLINK_YELLOW = 5
    COORDINATED = 1


class TrafficLightNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Parameter deklarieren
        # erlaubt: "red", "yellow", "green", optional "off"
        self.declare_parameter('color-red', (255, 0, 0))
        self.declare_parameter('color-yellow', (255, 255, 0))
        self.declare_parameter('colcor-green', (0, 255, 0))
        self.declare_parameter('timeout', 120)
        self.declare_parameter('mode', 0)
        
        #Publisher: Ampel-State und Ampel-Farbe

        #Blink-Frequenz
        #

        #Services: 
        #          Function: Standarbetrieb, mit Rot/Gelb Phase, Außer Betrieb (blinkt Gelb), OFF, RED, GREEN, YELLOW
        #           
        #          Service: Alaram (Schaltet alle anderen Ampelen auf Rot und sprint dann selbst auf Grün)
        #          Abfrage:
        #          SingleLED: ON/OFF(Standard)

        # LED-Konfiguration
        led_count = 3  # 0=rot, 1=gelb, 2=grün
        spi_bus = 0
        spi_dev = 0

        # SK6812 -> 4 Bytes (RGBW)
        self.pixels = np.NeoPixelSpiDev(
            bus=spi_bus,
            dev=spi_dev,
            n=led_count,
            pixel_order=np.GRB,
            bpp=3,
            auto_write=False
        )

        self.set_light_from_param()

        # optional: Parameter-Callback, falls du später dynamisch ändern willst
        self.add_on_set_parameters_callback(self.on_param_change)

    def on_param_change(self, params):
        for p in params:
            if p.name == 'color':
                self.get_logger().info(f'Parameter color geändert auf: {p.value}')
                self.set_light(p.value)
        return rclpy.parameter.SetParametersResult(successful=True)

    def set_light_from_param(self):
        color = self.get_parameter('color').get_parameter_value().string_value
        self.set_light(color)

    def set_light(self, color: str):
        # alles erstmal aus
        for i in range(len(self.pixels)):
            self.pixels[i] = (0, 0, 0)

        color = color.lower()

        # einfache Ampellogik: genau eine LED leuchtet
        if color == 'red':
            # GRBW -> (G, R, B, W)
            self.pixels[0] = (255, 0, 0)
        elif color in ('yellow', 'amber'):
            # gelb = rot + grün, hier leicht
            self.pixels[1] = (255, 255, 0)
        elif color == 'green':
            self.pixels[2] = (0, 255, 0)
        elif color == 'off':
            pass
        else:
            self.get_logger().warn(f'Unbekannte Ampelfarbe: {color}, schalte aus.')

        self.pixels.show()
        self.get_logger().info(f'Ampel auf "{color}" gesetzt.')



    def destroy_node(self):
        self.pixels.fill((0,0,0))
        self.pixels.show()
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TrafficLightNode("traffic_light")
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

if __name__ == "__main__":
    main()