;; Auto-generated. Do not edit!


(when (boundp 'robot_control::GetBaseLinkPose)
  (if (not (find-package "ROBOT_CONTROL"))
    (make-package "ROBOT_CONTROL"))
  (shadow 'GetBaseLinkPose (find-package "ROBOT_CONTROL")))
(unless (find-package "ROBOT_CONTROL::GETBASELINKPOSE")
  (make-package "ROBOT_CONTROL::GETBASELINKPOSE"))
(unless (find-package "ROBOT_CONTROL::GETBASELINKPOSEREQUEST")
  (make-package "ROBOT_CONTROL::GETBASELINKPOSEREQUEST"))
(unless (find-package "ROBOT_CONTROL::GETBASELINKPOSERESPONSE")
  (make-package "ROBOT_CONTROL::GETBASELINKPOSERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass robot_control::GetBaseLinkPoseRequest
  :super ros::object
  :slots (_joint_angles_branch1 _joint_angles_branch4 ))

(defmethod robot_control::GetBaseLinkPoseRequest
  (:init
   (&key
    ((:joint_angles_branch1 __joint_angles_branch1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_angles_branch4 __joint_angles_branch4) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_angles_branch1 __joint_angles_branch1)
   (setq _joint_angles_branch4 __joint_angles_branch4)
   self)
  (:joint_angles_branch1
   (&optional __joint_angles_branch1)
   (if __joint_angles_branch1 (setq _joint_angles_branch1 __joint_angles_branch1)) _joint_angles_branch1)
  (:joint_angles_branch4
   (&optional __joint_angles_branch4)
   (if __joint_angles_branch4 (setq _joint_angles_branch4 __joint_angles_branch4)) _joint_angles_branch4)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_angles_branch1
    (* 8    (length _joint_angles_branch1)) 4
    ;; float64[] _joint_angles_branch4
    (* 8    (length _joint_angles_branch4)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_angles_branch1
     (write-long (length _joint_angles_branch1) s)
     (dotimes (i (length _joint_angles_branch1))
       (sys::poke (elt _joint_angles_branch1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_angles_branch4
     (write-long (length _joint_angles_branch4) s)
     (dotimes (i (length _joint_angles_branch4))
       (sys::poke (elt _joint_angles_branch4 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_angles_branch1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles_branch1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles_branch1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_angles_branch4
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles_branch4 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles_branch4 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass robot_control::GetBaseLinkPoseResponse
  :super ros::object
  :slots (_base_link_pose ))

(defmethod robot_control::GetBaseLinkPoseResponse
  (:init
   (&key
    ((:base_link_pose __base_link_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _base_link_pose __base_link_pose)
   self)
  (:base_link_pose
   (&rest __base_link_pose)
   (if (keywordp (car __base_link_pose))
       (send* _base_link_pose __base_link_pose)
     (progn
       (if __base_link_pose (setq _base_link_pose (car __base_link_pose)))
       _base_link_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _base_link_pose
    (send _base_link_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _base_link_pose
       (send _base_link_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _base_link_pose
     (send _base_link_pose :deserialize buf ptr-) (incf ptr- (send _base_link_pose :serialization-length))
   ;;
   self)
  )

(defclass robot_control::GetBaseLinkPose
  :super ros::object
  :slots ())

(setf (get robot_control::GetBaseLinkPose :md5sum-) "124f2f1fb4bf270787e14bd14a8d0a4b")
(setf (get robot_control::GetBaseLinkPose :datatype-) "robot_control/GetBaseLinkPose")
(setf (get robot_control::GetBaseLinkPose :request) robot_control::GetBaseLinkPoseRequest)
(setf (get robot_control::GetBaseLinkPose :response) robot_control::GetBaseLinkPoseResponse)

(defmethod robot_control::GetBaseLinkPoseRequest
  (:response () (instance robot_control::GetBaseLinkPoseResponse :init)))

(setf (get robot_control::GetBaseLinkPoseRequest :md5sum-) "124f2f1fb4bf270787e14bd14a8d0a4b")
(setf (get robot_control::GetBaseLinkPoseRequest :datatype-) "robot_control/GetBaseLinkPoseRequest")
(setf (get robot_control::GetBaseLinkPoseRequest :definition-)
      "float64[] joint_angles_branch1
float64[] joint_angles_branch4

---
# Response: 位姿
geometry_msgs/Pose base_link_pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get robot_control::GetBaseLinkPoseResponse :md5sum-) "124f2f1fb4bf270787e14bd14a8d0a4b")
(setf (get robot_control::GetBaseLinkPoseResponse :datatype-) "robot_control/GetBaseLinkPoseResponse")
(setf (get robot_control::GetBaseLinkPoseResponse :definition-)
      "float64[] joint_angles_branch1
float64[] joint_angles_branch4

---
# Response: 位姿
geometry_msgs/Pose base_link_pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :robot_control/GetBaseLinkPose "124f2f1fb4bf270787e14bd14a8d0a4b")


