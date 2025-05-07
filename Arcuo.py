import cv2
import numpy as np

print("OpenCV 版本:", cv2.__version__)

# 设置 ArUco 字典和参数
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# 相机参数（内参矩阵）
mtx = np.array([[602.85, 0., 324.607],
                [0., 601.703, 238.165],
                [0., 0., 1.]])
dist = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# 打开 RGB 摄像头
cap = cv2.VideoCapture(6)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取帧")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # 姿态估计：不要 import，只直接调用
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
            corners, 0.035, mtx, dist
        )

        for i in range(len(ids)):
            rmat, _ = cv2.Rodrigues(rvecs[i])
            pose_matrix = np.eye(4)
            pose_matrix[:3, :3] = rmat
            pose_matrix[:3, 3] = tvecs[i].flatten()

            print(f'Marker ID: {ids[i][0]}')
            print(f'4x4 Pose Matrix:\n{pose_matrix}')

            # 坐标轴可视化
            cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.1)

            with open("center_pose.txt", "a") as file:
                file.write(f"{pose_matrix}\n")

    cv2.imshow('ARUCO Marker Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
# import cv2
# import numpy as np

# # 检查 OpenCV 版本
# print("OpenCV 版本:", cv2.__version__)

# # 定义 ARUCO 字典
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# # 定义参数检测器
# parameters = cv2.aruco.DetectorParameters()

# # 相机内参矩阵（假设没有畸变）
# mtx = np.array([[602.85, 0., 324.607],
#                 [0., 601.703, 238.165],
#                 [0., 0., 1.]])
# dist = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# # 打开摄像头
# cap = cv2.VideoCapture(0)
# if not cap.isOpened():
#     print("无法打开摄像头")
#     exit()

# while True:
#     # 读取帧
#     ret, frame = cap.read()
#     if not ret:
#         print("无法读取帧")
#         break

#     # 转换为灰度图像
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # 检测 ARUCO 标记
#     corners, ids, rejectedImgPoints = cv2.aruco.ArucoDetector.detectMarkers(gray, aruco_dict, parameters=parameters)

#     # 如果检测到标记
#     if ids is not None:
#         # 绘制检测到的标记
#         frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

#         # 估计标记的位姿
#         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.035, mtx, dist)

#         for i in range(len(ids)):
#             # 将旋转向量转换为旋转矩阵
#             rmat, _ = cv2.Rodrigues(rvecs[i])

#             # 构建 4x4 的位姿矩阵
#             pose_matrix = np.eye(4)  # 创建一个 4x4 的单位矩阵
#             pose_matrix[:3, :3] = rmat  # 填充旋转矩阵
#             pose_matrix[:3, 3] = tvecs[i].flatten()  # 填充平移向量

#             # 输出位姿矩阵
#             print(f'Marker ID: {ids[i][0]}')
#             print(f'4x4 Pose Matrix:\n{pose_matrix}')

#             # 绘制坐标轴
#             cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.1)

#             with open("center_pose.txt","a") as file:
#                 file.write(f"{pose_matrix}\n")

#     # 显示结果
#     cv2.imshow('ARUCO Marker Detection', frame)

#     # 按 'q' 键退出
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # 释放摄像头并关闭所有窗口
# cap.release()
# cv2.destroyAllWindows()

