;; Auto-generated. Do not edit!


(when (boundp 'robot_planning::RobotPose)
  (if (not (find-package "ROBOT_PLANNING"))
    (make-package "ROBOT_PLANNING"))
  (shadow 'RobotPose (find-package "ROBOT_PLANNING")))
(unless (find-package "ROBOT_PLANNING::ROBOTPOSE")
  (make-package "ROBOT_PLANNING::ROBOTPOSE"))
(unless (find-package "ROBOT_PLANNING::ROBOTPOSEREQUEST")
  (make-package "ROBOT_PLANNING::ROBOTPOSEREQUEST"))
(unless (find-package "ROBOT_PLANNING::ROBOTPOSERESPONSE")
  (make-package "ROBOT_PLANNING::ROBOTPOSERESPONSE"))

(in-package "ROS")





(defclass robot_planning::RobotPoseRequest
  :super ros::object
  :slots (_float_base_pose _branch2_joints _branch3_joints _source_frame _target_frame ))

(defmethod robot_planning::RobotPoseRequest
  (:init
   (&key
    ((:float_base_pose __float_base_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:branch2_joints __branch2_joints) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:branch3_joints __branch3_joints) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:source_frame __source_frame) "")
    ((:target_frame __target_frame) "")
    )
   (send-super :init)
   (setq _float_base_pose __float_base_pose)
   (setq _branch2_joints __branch2_joints)
   (setq _branch3_joints __branch3_joints)
   (setq _source_frame (string __source_frame))
   (setq _target_frame (string __target_frame))
   self)
  (:float_base_pose
   (&optional __float_base_pose)
   (if __float_base_pose (setq _float_base_pose __float_base_pose)) _float_base_pose)
  (:branch2_joints
   (&optional __branch2_joints)
   (if __branch2_joints (setq _branch2_joints __branch2_joints)) _branch2_joints)
  (:branch3_joints
   (&optional __branch3_joints)
   (if __branch3_joints (setq _branch3_joints __branch3_joints)) _branch3_joints)
  (:source_frame
   (&optional __source_frame)
   (if __source_frame (setq _source_frame __source_frame)) _source_frame)
  (:target_frame
   (&optional __target_frame)
   (if __target_frame (setq _target_frame __target_frame)) _target_frame)
  (:serialization-length
   ()
   (+
    ;; float64[] _float_base_pose
    (* 8    (length _float_base_pose)) 4
    ;; float64[] _branch2_joints
    (* 8    (length _branch2_joints)) 4
    ;; float64[] _branch3_joints
    (* 8    (length _branch3_joints)) 4
    ;; string _source_frame
    4 (length _source_frame)
    ;; string _target_frame
    4 (length _target_frame)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _float_base_pose
     (write-long (length _float_base_pose) s)
     (dotimes (i (length _float_base_pose))
       (sys::poke (elt _float_base_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _branch2_joints
     (write-long (length _branch2_joints) s)
     (dotimes (i (length _branch2_joints))
       (sys::poke (elt _branch2_joints i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _branch3_joints
     (write-long (length _branch3_joints) s)
     (dotimes (i (length _branch3_joints))
       (sys::poke (elt _branch3_joints i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _source_frame
       (write-long (length _source_frame) s) (princ _source_frame s)
     ;; string _target_frame
       (write-long (length _target_frame) s) (princ _target_frame s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _float_base_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _float_base_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _float_base_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _branch2_joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _branch2_joints (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _branch2_joints i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _branch3_joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _branch3_joints (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _branch3_joints i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _source_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _target_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot_planning::RobotPoseResponse
  :super ros::object
  :slots (_transform _success _message ))

(defmethod robot_planning::RobotPoseResponse
  (:init
   (&key
    ((:transform __transform) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _transform __transform)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:transform
   (&optional __transform)
   (if __transform (setq _transform __transform)) _transform)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; float64[] _transform
    (* 8    (length _transform)) 4
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _transform
     (write-long (length _transform) s)
     (dotimes (i (length _transform))
       (sys::poke (elt _transform i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _transform
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _transform (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _transform i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot_planning::RobotPose
  :super ros::object
  :slots ())

(setf (get robot_planning::RobotPose :md5sum-) "88bd561bfe99b318bb370cf7b0dd2f58")
(setf (get robot_planning::RobotPose :datatype-) "robot_planning/RobotPose")
(setf (get robot_planning::RobotPose :request) robot_planning::RobotPoseRequest)
(setf (get robot_planning::RobotPose :response) robot_planning::RobotPoseResponse)

(defmethod robot_planning::RobotPoseRequest
  (:response () (instance robot_planning::RobotPoseResponse :init)))

(setf (get robot_planning::RobotPoseRequest :md5sum-) "88bd561bfe99b318bb370cf7b0dd2f58")
(setf (get robot_planning::RobotPoseRequest :datatype-) "robot_planning/RobotPoseRequest")
(setf (get robot_planning::RobotPoseRequest :definition-)
      "float64[] float_base_pose  # [x, y, z, qx, qy, qz, qw] 相对于world坐标系的位姿
float64[] branch2_joints   # 分支2的关节角
float64[] branch3_joints   # 分支3的关节角
string source_frame        # 源坐标系名称
string target_frame        # 目标坐标系名称
---
float64[] transform       # [x, y, z, qx, qy, qz, qw] 源坐标系到目标坐标系的变换
bool success
string message 
")

(setf (get robot_planning::RobotPoseResponse :md5sum-) "88bd561bfe99b318bb370cf7b0dd2f58")
(setf (get robot_planning::RobotPoseResponse :datatype-) "robot_planning/RobotPoseResponse")
(setf (get robot_planning::RobotPoseResponse :definition-)
      "float64[] float_base_pose  # [x, y, z, qx, qy, qz, qw] 相对于world坐标系的位姿
float64[] branch2_joints   # 分支2的关节角
float64[] branch3_joints   # 分支3的关节角
string source_frame        # 源坐标系名称
string target_frame        # 目标坐标系名称
---
float64[] transform       # [x, y, z, qx, qy, qz, qw] 源坐标系到目标坐标系的变换
bool success
string message 
")



(provide :robot_planning/RobotPose "88bd561bfe99b318bb370cf7b0dd2f58")


