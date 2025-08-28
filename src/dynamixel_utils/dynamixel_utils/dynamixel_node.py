import rclpy
from rclpy.node import Node

from dynamixel_utils.dynamixel import Dynamixel

def main_dxl_scan(args=None):
    rclpy.init(args=args)
    mynode:Node= Node("dxl_scan")
    mynode.declare_parameters("", [("uart_name", ""), ("baudrate", 57600), ("min_id", 1), ("max_id", 5)])
    try:
        mynode.get_logger().info("Scanning for Dynamixel motors!")
        uart_name = mynode.get_parameter("uart_name").get_parameter_value().string_value
        baudrate = mynode.get_parameter("baudrate").get_parameter_value().integer_value
        min_id = mynode.get_parameter("min_id").get_parameter_value().integer_value
        max_id = mynode.get_parameter("max_id").get_parameter_value().integer_value

        mynode.get_logger().info(f"Received parameters: uart_name={uart_name}, baudrate={baudrate}, min_id={min_id}, max_id={max_id}")

        dynamixel_scanner = Dynamixel(Dynamixel.Config(baudrate=baudrate, device_name=uart_name))
        motor_ids = dynamixel_scanner.scan(uart_name, baudrate, min_id, max_id)
        dynamixel_scanner.disconnect()
        if motor_ids:
            mynode.get_logger().info(f"Found Dynamixel motors with IDs: {motor_ids}")
        else:
            mynode.get_logger().warn("No Dynamixel motors found.") 
         
    except KeyboardInterrupt:
        pass
    except Exception as e:
        mynode.get_logger().error(f"An error occurred: {e}")
    finally:
        if mynode is not None:
            if rclpy.ok():
                mynode.get_logger().info(f"Node {mynode.get_name()} wird beendet!")
            mynode.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_dxl_set_id(args=None):
    rclpy.init(args=args)
    mynode:Node = Node("dxl_set_id")
    mynode.declare_parameters("", [("uart_name", "/dev/ttyACM0"), ("baudrate", 57600), ("id", 254), ("new_id", 1)])
    try:
        mynode.get_logger().info("Setting Dynamixel motor ID!")
        uart_name = mynode.get_parameter("uart_name").get_parameter_value().string_value
        baudrate = mynode.get_parameter("baudrate").get_parameter_value().integer_value
        motor_id = mynode.get_parameter("id").get_parameter_value().integer_value
        new_id = mynode.get_parameter("new_id").get_parameter_value().integer_value
        mynode.get_logger().info(f"Received parameters: uart_name={uart_name}, baudrate={baudrate}, id={motor_id}, new_id={new_id}")
        dxlctrl = Dynamixel(Dynamixel.Config(baudrate=baudrate, device_name=uart_name))
        dxlctrl.set_id(motor_id, new_id)
        dxlctrl.disconnect()
        mynode.get_logger().info(f"Motor ID successfully changed from {motor_id} to {new_id} on {uart_name}.")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        mynode.get_logger().error(f"An error occurred: {e}")
    finally:
        if mynode is not None:
            if rclpy.ok():
                mynode.get_logger().info(f"Node {mynode.get_name()} wird beendet!")
            mynode.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
