import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
 
class Streaming(Node):

  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      '/camera', 
      self.listener_callback, 
      10)
    self.subscription 
    self.bridge = CvBridge()

  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    current_frame = self.bridge.imgmsg_to_cv2(data)
    model = YOLO("./utils/best.pt")
    result = model.predict(current_frame, conf=0.6)
    annotated = result[0].plot()
    cv2.imshow("camera", annotated)
    cv2.waitKey(1)

