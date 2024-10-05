import rclpy
from modules import Camera


def main(args=None):
    rclpy.init()
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()