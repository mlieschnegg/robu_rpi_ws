import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

import cv2

# Definiere den GStreamer-Pipeline-String
# Die Kamerauflösung muss auf meine Kamera angepasst werden
gstreamer_pipeline = (
    "libcamerasrc ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! "
    "appsink"
)

class CameraImagePub(Node):
    def __init__(self, node_name='camera_iamge_pub'):
        super().__init__(node_name)

        self._initialized = False

        #Publish to the camera_line_detection topic
        self._pub_image_raw = self.create_publisher(Image, 'camera/image_raw', 10)

        self._cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            err_msg = "Kamera konnte nicht geöffnet werden!"
            self.get_logger().error(err_msg)
            raise Exception(err_msg)

        self._bridge = CvBridge()

        self._initialized = True
        self._timer_capture_image = self.create_timer(0.05, self._cb_pub_camera_image)

    def _cb_pub_camera_image(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn("Fehler beim Einlesen des Frames.")
            return

        # cv2.imwrite("image_raw.jpg", frame)
        self._pub_image_raw.publish(self._bridge.cv2_to_imgmsg(frame, "bgr8"))

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap.isOpened():
            self._cap.release()
            self.get_logger().info("Kamera wurde freigegeben.")
        super().destroy_node()


def main():
    try:
        rclpy.init()
        node = CameraImagePub("camera_image_pub")
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()