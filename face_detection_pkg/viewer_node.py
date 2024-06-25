#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Viewer(Node):

    def __init__(self):
        super().__init__('viewer_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'unibas_face_distance_calculator/faces',
            self.callback,
            10
        )

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')

        cv2.imshow('faces + depth', cv_image)
        cv2.waitKey(30)

def main(args=None):
    rclpy.init(args=args)
    viewer = Viewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print('Shutting down')
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
