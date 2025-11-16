#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import os
import numpy as np   # ✅ 이거 추가!!!

class SyncGT:
    def __init__(self):
        rospy.init_node('sync_gt_node', anonymous=True)

        image_topic = rospy.get_param("~image_topic", "/zed/left/image_rect_color/compressed")
        odom_topic  = rospy.get_param("~odom_topic", "/zed/odom")
        self.save_dir = rospy.get_param("~save_dir", "/home/jairlab/Downloads/zed_sync_images")
        self.out_txt  = rospy.get_param("~out_txt", "/home/jairlab/Downloads/matched_gt_sync.txt")

        os.makedirs(self.save_dir, exist_ok=True)
        self.f = open(self.out_txt, "w")
        self.bridge = CvBridge()
        self.count = 0

        # message_filters 기반 동기화
        sub_img = message_filters.Subscriber(image_topic, CompressedImage)
        sub_odom = message_filters.Subscriber(odom_topic, Odometry)

        ts = message_filters.ApproximateTimeSynchronizer(
            [sub_img, sub_odom],
            queue_size=100,
            slop=0.1
        )
        ts.registerCallback(self.callback)

        rospy.loginfo("✅ sync_gt_node started!")
        rospy.spin()

    def callback(self, img_msg, odom_msg):
        try:
            # 이미지 저장
            frame_name = f"frame{self.count:06d}.jpg"
            frame_path = os.path.join(self.save_dir, frame_name)

            np_arr = np.frombuffer(img_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imwrite(frame_path, cv_image)

            # 오도메트리 pose 기록
            pos = odom_msg.pose.pose.position
            ori = odom_msg.pose.pose.orientation
            self.f.write(f"{frame_name} {pos.x} {pos.y} {pos.z} {ori.x} {ori.y} {ori.z} {ori.w}\n")

            rospy.loginfo(f"[INFO] Saved {frame_name}")
            self.count += 1
        except Exception as e:
            rospy.logwarn(f"Callback error: {e}")

if __name__ == "__main__":
    SyncGT()

