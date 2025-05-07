#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import yaml
import os
import cv2
import numpy as np
import rospkg
from std_srvs.srv import Trigger, TriggerResponse

class ArucoPoseServer:
    def __init__(self):
        rospy.init_node("aruco_pose_service")
        self.cap = cv2.VideoCapture(6)
        if not self.cap.isOpened():
            rospy.logerr("Unable to open camera")
            exit()

        # 相机内参
        self.camera_matrix = np.array([[602.85, 0., 324.607],
                                       [0., 601.703, 238.165],
                                       [0., 0., 1.]])
        self.dist_coeffs = np.array([0., 0., 0., 0.], dtype=np.float32)
        self.marker_length = 0.035  # 单位：米

        # ArUco 设置
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # YAML 保存路径
        rospack = rospkg.RosPack()
        config_dir = os.path.join(rospack.get_path("robot_vision"), "config")
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
        self.yaml_path = os.path.join(config_dir, "aruco_pose.yaml")

        self.service = rospy.Service("/detect_aruco_and_save", Trigger, self.handle_request)
        rospy.loginfo("Service started:/detect_aruco_and_save")

    def handle_request(self, req):
        ret, frame = self.cap.read()
        if not ret:
            return TriggerResponse(success=False, message="图像采集失败")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return TriggerResponse(success=False, message="未检测到 ArUco 标记")

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        # 只处理第一个 ArUco
        rmat, _ = cv2.Rodrigues(rvecs[0])
        position = [float(x) for x in tvecs[0].flatten()]
        orientation = [[float(rmat[i, j]) for j in range(3)] for i in range(3)]

        result = {
            "tf_mat_camera_obj": {
                "target_pose": {
                    "position": position,
                    "orientation": orientation
                }
            }
        }

        # 写入 YAML，确保 flow style（列表在一行）
        with open(self.yaml_path, "w") as f:
            yaml.dump(result, f, sort_keys=False, default_flow_style=None)

        rospy.loginfo("检测完成，已保存至：%s", self.yaml_path)
        return TriggerResponse(success=True, message="检测完成，已保存 ArUco 位姿")


    def shutdown(self):
        self.cap.release()

if __name__ == "__main__":
    try:
        server = ArucoPoseServer()
        rospy.spin()
    finally:
        server.shutdown()
