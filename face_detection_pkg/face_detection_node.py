import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class GetPersonDetection(Node):

    def __init__(self):
        super().__init__('person_detection')
        self.bridge = CvBridge()

        # Initialize subscribers
        self.image_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")

        # Point x, y, distance publisher
        self.point_pub = self.create_publisher(Point, '/person_point', 10)

        # Synchronize subscriptions
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.callback)

        # Locate YOLOv3 model and configuration files using package share directory
        package_share_directory = get_package_share_directory('face_detection_pkg')
        config_dir = os.path.join(package_share_directory, 'config')

        # Load YOLO model
        yolov3_cfg = os.path.join(config_dir, 'yolov3.cfg')
        yolov3_weights = os.path.join(config_dir, 'yolov3.weights')
        coco_names = os.path.join(config_dir, 'coco.names')

        self.net = cv2.dnn.readNetFromDarknet(yolov3_cfg, yolov3_weights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        
        # Read COCO class names
        self.classes = []
        with open(coco_names, 'r') as f:
            self.classes = f.read().strip().split('\n')


    def callback(self, rgb_data, depth_data):

        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        trackers = []
        tracker_labels = []
        confidence_threshold = 0.6
        nms_threshold = 0.4

        try:
            # Convert ROS messages to OpenCV format
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            height, width, _ = cv_rgb.shape
            # print(height)
            # print(width)
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
    
            # Resize frame for YOLO
            frame_resized = cv2.resize(cv_rgb, (320, 320))

            # Resize depth image to match RGB image
            depth_image_resized = cv2.resize(depth_image, (cv_rgb.shape[1], cv_rgb.shape[0]))
            

            # Normalize depth image
            depth_array = np.array(depth_image_resized, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            depth_8 = (depth_array * 255).round().astype(np.uint8)
            
            # Create RGBD image (for visualization)
            cv_depth = np.zeros_like(cv_rgb)
            cv_depth[:, :, 0] = depth_8
            cv_depth[:, :, 1] = depth_8
            cv_depth[:, :, 2] = depth_8

            # Detecting objects using YOLO
            blob = cv2.dnn.blobFromImage(cv_rgb, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(output_layers)

            class_ids = []
            confidences = []
            boxes = []

            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > confidence_threshold and self.classes[class_id] == 'person':
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append((x, y, w, h))
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold)
            updated_boxes = []

            if len(indexes) > 0:
                for i in indexes.flatten():
                    updated_boxes.append(boxes[i])

            # Update trackers with the new detections
            trackers = []
            tracker_labels = []

            for bbox in updated_boxes:
                tracker = cv2.legacy.TrackerKCF_create()
                trackers.append(tracker)
                tracker.init(cv_rgb, bbox)
                tracker_labels.append(f'Person {len(tracker_labels) + 1}')

            for i, tracker in enumerate(trackers):
                success, box = tracker.update(cv_rgb)
                if success:
                    x, y, w, h = [int(v) for v in box]
                    center = (x + w//2, y + h//2)
                    distance = depth_image_resized[center[1], center[0]] * 0.001 # Convert to meters
                    label = tracker_labels[i]
                    color = (0, 255, 0)

                    # Message to publish
                    point_msg = Point()
                    point_msg.x = float(center[0])
                    point_msg.y = float(center[1])
                    point_msg.z = distance # in meters
                    self.point_pub.publish(point_msg)

                    # Debug visualization

                    cv2.circle(cv_rgb, (center[0], center[1]), 5, (0, 0, 255), 2)
                    cv2.rectangle(cv_rgb, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(cv_rgb, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(cv_rgb, f"Distance: {distance:.2f} m", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow('Webcam YOLO Person Detection', cv_rgb)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

   

def main(args=None):
    rclpy.init(args=args)
    pd = GetPersonDetection()
    try:
        rclpy.spin(pd)
    except KeyboardInterrupt:
        pass
    finally:
        pd.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
