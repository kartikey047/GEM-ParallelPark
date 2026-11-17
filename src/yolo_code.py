#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO


class ParkingSpaceDetector(Node):
    def __init__(self):
        super().__init__('parking_space_detector')

        self.model = YOLO("Cone.pt").to("cpu")

        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, 'space_detection', 10)

        self.sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )
        self.iter = 0
        self.get_logger().info("Parking Space Detector Node Started.")
    def image_callback(self, msg):
        self.iter+=1
        if(self.iter%5 != 0):
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = img.shape[:2]
        results = self.model.predict(source=img, conf=0.25, save=False, verbose=False)
        cone_points = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                cx = int((x1 + x2) / 2)
                cy = int(y2)
                cone_points.append((cx, cy))
        clone = []
        for p in cone_points:
            is_good = True
            for q in cone_points:
                if (p != q) and ((p[0]-q[0])**2 + (p[1]-q[1])**2 <= 5**2):
                    is_good = False
                    break
            if is_good:
                clone.append(p)
        cone_points = sorted(clone, key=lambda p: p[0])
        overlay = img.copy()
        n = len(cone_points)
        if n == 4: #First try to arrange them counterclockwise along their center
            quadrilateral = np.array(cone_points, np.int32)
            centroid = quadrilateral.mean(axis=0)
            angles = np.arctan2(quadrilateral[:,1] - centroid[1],
                                quadrilateral[:,0] - centroid[0])
            order = np.argsort(angles)
            quadrilateral = quadrilateral[order].astype(np.int32)
            area = cv2.contourArea(quadrilateral) # then try making the rectangle
            if area < 0.2:
                print("Too Far from Space")
            else:
                cv2.fillPoly(overlay, [quadrilateral], (0, 255, 255))
                cv2.polylines(img, [quadrilateral], isClosed=True,
                            color=(0, 255, 255), thickness=2)
        elif n == 3:
            cv2.fillPoly(overlay, [np.array(cone_points, np.int32)], (0, 255, 255))
            cv2.polylines(img, [np.array(cone_points, np.int32)], True,
                        (0, 255, 255), 2)
        else:
            print("Not enough data to determine parking space location, ",
                n, " cones found, need minimum 3 cones to determine parking space location")
        alpha = 0.3
        img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
        for (cx, cy) in cone_points:
            cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub.publish(out_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()