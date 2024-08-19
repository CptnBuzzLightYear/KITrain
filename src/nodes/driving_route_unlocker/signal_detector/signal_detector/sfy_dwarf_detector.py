import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import pandas as pd
from std_msgs.msg import Bool, String

# Load class names
with open('/home/steffens/KITrain/src/nodes/driving_route_unlocker/signal_detector/signal_detector/class_names.txt', "r") as my_file:
    data = my_file.read()
class_list = data.split("\n")
print(class_list)

class VideoSubscriber(Node):

    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/received_image',
            self.video_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'dwarf_signal_detection', 10)
        
        # New Publishers for signal detection and setting
        self.signal_detected_publisher = self.create_publisher(Bool, 'signal_detected', 10)
        self.signal_setting_publisher = self.create_publisher(String, 'signal_setting', 10)
        
        self.bridge = CvBridge()
        self.model = YOLO("/home/steffens/KITrain/src/nodes/driving_route_unlocker/signal_detector/signal_detector/best.pt")  # Ensure 'best.pt' is in the src directory
        self.get_logger().info("Video subscriber node has been started.")

    def video_callback(self, ros_frame):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_frame, "bgr8")
            results = self.model.predict(frame)
            a = results[0].boxes.data.tolist()
            px = pd.DataFrame(a).astype("float")
            
            signal_detected = False
            signal_setting = ""

            for index, row in px.iterrows():
                x1 = int(row[0])
                y1 = int(row[1])
                x2 = int(row[2])
                y2 = int(row[3])
                d = int(row[5])  # Signal class: d=0 (white), d=1 (red)
                c = class_list[d]  # Class name

                if d == 0:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
                    signal_detected = True
                    signal_setting = "red"
                else:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (242, 242, 242), 2)
                    cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (139, 134, 130), 1)
                    signal_detected = True
                    signal_setting = "white"

            # Publish the detection status
            self.signal_detected_publisher.publish(Bool(data=signal_detected))

            # Publish the signal setting
            self.signal_setting_publisher.publish(String(data=signal_setting))

            # Convert the modified frame back to a ROS Image message and publish it
            ros_frame_out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_frame_out)
        
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# from ultralytics import YOLO
# import pandas as pd

# # Load class names
# with open('/home/steffens/KITrain/src/nodes/driving_route_unlocker/signal_detector/signal_detector/class_names.txt', "r") as my_file:
#     data = my_file.read()
# class_list = data.split("\n")
# print(class_list)

# class VideoSubscriber(Node):

#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/received_image',
#             self.video_callback,
#             10)
#         self.publisher_ = self.create_publisher(Image, 'dwarf_signal_detection', 10)
#         self.bridge = CvBridge()
#         self.model = YOLO("/home/steffens/KITrain/src/nodes/driving_route_unlocker/signal_detector/signal_detector/best.pt")  # Ensure 'best.pt' is in the src directory
#         self.get_logger().info("Video subscriber node has been started.")

#     def video_callback(self, ros_frame):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(ros_frame, "bgr8")
#             results = self.model.predict(frame)
#             a = results[0].boxes.data.tolist()
#             px = pd.DataFrame(a).astype("float")
#             for index, row in px.iterrows():
#                 x1 = int(row[0])
#                 y1 = int(row[1])
#                 x2 = int(row[2])
#                 y2 = int(row[3])
#                 d = int(row[5]) # klasse, d=0: sh1 (white), d=1 : hp0 (red)
#                 c = class_list[d] # klassenbezeichnung
#                 # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
#                 # cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)
#                 if d == 0:
#                     cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
#                     cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
#                 else:
#                     cv2.rectangle(frame, (x1, y1), (x2, y2), (242, 242, 242), 2)
#                     cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (139, 134, 130), 1)
            
#             # Convert the modified frame back to a ROS Image message and publish it
#             ros_frame_out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
#             self.publisher_.publish(ros_frame_out)
        
#         except CvBridgeError as e:
#             self.get_logger().error(f"CvBridge Error: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     video_subscriber = VideoSubscriber()
#     rclpy.spin(video_subscriber)
#     video_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
