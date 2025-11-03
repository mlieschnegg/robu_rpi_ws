from rclpy.node import Node
import rclpy

from neopixel.ledlib import neopixel_spidev as np

class TrafficLightNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)


        # Parameter deklarieren
        # erlaubt: "red", "yellow", "green", optional "off"
        self.declare_parameter('color', 'red')

        # LED-Konfiguration
        led_count = 3  # 0=rot, 1=gelb, 2=grün
        spi_bus = 0
        spi_dev = 0

        # SK6812 -> 4 Bytes (GRBW)
        self.pixels = np.NeoPixelSpiDev(
            bus=spi_bus,
            dev=spi_dev,
            n=led_count,
            pixel_order=np.GRBW,
            bpp=4
        )

        # beim Start direkt Farbe setzen
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
            self.pixels[i] = (0, 0, 0, 0)

        color = color.lower()

        # einfache Ampellogik: genau eine LED leuchtet
        if color == 'red':
            # GRBW -> (G, R, B, W)
            self.pixels[0] = (0, 255, 0, 0)
        elif color in ('yellow', 'amber'):
            # gelb = rot + grün, hier leicht
            self.pixels[1] = (50, 255, 0, 0)
        elif color == 'green':
            self.pixels[2] = (255, 0, 0, 0)
        elif color == 'off':
            pass  # alles bleibt aus
        else:
            self.get_logger().warn(f'Unbekannte Ampelfarbe: {color}, schalte aus.')

        self.pixels.show()
        self.get_logger().info(f'Ampel auf "{color}" gesetzt.')



    def destroy_node(self):
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