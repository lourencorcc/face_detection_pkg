import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class GetPointDistance(Node):

    def __init__(self):
        super().__init__('point_distance_calculator')
        self.bridge = CvBridge()

        # Initialize subscribers
        self.image_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")

        # Synchronize subscriptions
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.callback)

    def callback(self, rgb_data, depth_data):
        try:
            # Convert ROS messages to OpenCV format
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
            
            # Resize depth image to match RGB image
            depth_image_resized = cv2.resize(depth_image, (cv_rgb.shape[1], cv_rgb.shape[0]))
            
            # Normalize depth image
            depth_array = np.array(depth_image_resized, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            depth_8 = (depth_array * 255).round().astype(np.uint8)
            
            # Create RGBD image
            cv_depth = np.zeros_like(cv_rgb)
            cv_depth[:, :, 0] = depth_8
            cv_depth[:, :, 1] = depth_8
            cv_depth[:, :, 2] = depth_8
            
            # Point of interest (0, 0)
            point_x = 800 # 1280
            point_y = 300 # 720

            # Extract depth value at the point
            distance = depth_image_resized[point_y, point_x] * 0.001  # Convert to meters

            # Draw circle around the point in both RGB and depth images
            cv2.circle(cv_rgb, (point_x, point_y), 5, (0, 0, 255), 2)
            cv2.circle(cv_depth, (point_x, point_y), 5, (0, 0, 255), 2)

            # Display distance text (modify position as needed)
            dist_str = f"Distance: {distance:.2f} m"
            cv2.putText(cv_rgb, dist_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)

            # Show the RGB and depth images
            cv2.imshow('RGB Image', cv_rgb)
            cv2.imshow('Depth Image', cv_depth)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    fd = GetPointDistance()
    try:
        rclpy.spin(fd)
    except KeyboardInterrupt:
        pass
    finally:
        fd.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
