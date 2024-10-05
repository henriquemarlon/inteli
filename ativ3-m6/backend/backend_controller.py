import rclpy
from modules import Streaming


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Streaming()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()