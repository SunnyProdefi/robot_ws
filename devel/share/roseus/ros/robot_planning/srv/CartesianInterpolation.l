;; Auto-generated. Do not edit!


(when (boundp 'robot_planning::CartesianInterpolation)
  (if (not (find-package "ROBOT_PLANNING"))
    (make-package "ROBOT_PLANNING"))
  (shadow 'CartesianInterpolation (find-package "ROBOT_PLANNING")))
(unless (find-package "ROBOT_PLANNING::CARTESIANINTERPOLATION")
  (make-package "ROBOT_PLANNING::CARTESIANINTERPOLATION"))
(unless (find-package "ROBOT_PLANNING::CARTESIANINTERPOLATIONREQUEST")
  (make-package "ROBOT_PLANNING::CARTESIANINTERPOLATIONREQUEST"))
(unless (find-package "ROBOT_PLANNING::CARTESIANINTERPOLATIONRESPONSE")
  (make-package "ROBOT_PLANNING::CARTESIANINTERPOLATIONRESPONSE"))

(in-package "ROS")





(defclass robot_planning::CartesianInterpolationRequest
  :super ros::object
  :slots (_branch_id _joint_angles _start_pose _goal_pose _duration _frequency ))

(defmethod robot_planning::CartesianInterpolationRequest
  (:init
   (&key
    ((:branch_id __branch_id) 0)
    ((:joint_angles __joint_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:start_pose __start_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:goal_pose __goal_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:duration __duration) 0.0)
    ((:frequency __frequency) 0.0)
    )
   (send-super :init)
   (setq _branch_id (round __branch_id))
   (setq _joint_angles __joint_angles)
   (setq _start_pose __start_pose)
   (setq _goal_pose __goal_pose)
   (setq _duration (float __duration))
   (setq _frequency (float __frequency))
   self)
  (:branch_id
   (&optional __branch_id)
   (if __branch_id (setq _branch_id __branch_id)) _branch_id)
  (:joint_angles
   (&optional __joint_angles)
   (if __joint_angles (setq _joint_angles __joint_angles)) _joint_angles)
  (:start_pose
   (&optional __start_pose)
   (if __start_pose (setq _start_pose __start_pose)) _start_pose)
  (:goal_pose
   (&optional __goal_pose)
   (if __goal_pose (setq _goal_pose __goal_pose)) _goal_pose)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:serialization-length
   ()
   (+
    ;; int32 _branch_id
    4
    ;; float64[] _joint_angles
    (* 8    (length _joint_angles)) 4
    ;; float64[] _start_pose
    (* 8    (length _start_pose)) 4
    ;; float64[] _goal_pose
    (* 8    (length _goal_pose)) 4
    ;; float64 _duration
    8
    ;; float64 _frequency
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _branch_id
       (write-long _branch_id s)
     ;; float64[] _joint_angles
     (write-long (length _joint_angles) s)
     (dotimes (i (length _joint_angles))
       (sys::poke (elt _joint_angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _start_pose
     (write-long (length _start_pose) s)
     (dotimes (i (length _start_pose))
       (sys::poke (elt _start_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _goal_pose
     (write-long (length _goal_pose) s)
     (dotimes (i (length _goal_pose))
       (sys::poke (elt _goal_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _frequency
       (sys::poke _frequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _branch_id
     (setq _branch_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _start_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _start_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _goal_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _goal_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _duration
     (setq _duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _frequency
     (setq _frequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass robot_planning::CartesianInterpolationResponse
  :super ros::object
  :slots (_joint_trajectory _success _message ))

(defmethod robot_planning::CartesianInterpolationResponse
  (:init
   (&key
    ((:joint_trajectory __joint_trajectory) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _joint_trajectory __joint_trajectory)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:joint_trajectory
   (&optional __joint_trajectory)
   (if __joint_trajectory (setq _joint_trajectory __joint_trajectory)) _joint_trajectory)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_trajectory
    (* 8    (length _joint_trajectory)) 4
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_trajectory
     (write-long (length _joint_trajectory) s)
     (dotimes (i (length _joint_trajectory))
       (sys::poke (elt _joint_trajectory i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_trajectory (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_trajectory i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot_planning::CartesianInterpolation
  :super ros::object
  :slots ())

(setf (get robot_planning::CartesianInterpolation :md5sum-) "704fa7f95d1d0a395781e62e40e60c7e")
(setf (get robot_planning::CartesianInterpolation :datatype-) "robot_planning/CartesianInterpolation")
(setf (get robot_planning::CartesianInterpolation :request) robot_planning::CartesianInterpolationRequest)
(setf (get robot_planning::CartesianInterpolation :response) robot_planning::CartesianInterpolationResponse)

(defmethod robot_planning::CartesianInterpolationRequest
  (:response () (instance robot_planning::CartesianInterpolationResponse :init)))

(setf (get robot_planning::CartesianInterpolationRequest :md5sum-) "704fa7f95d1d0a395781e62e40e60c7e")
(setf (get robot_planning::CartesianInterpolationRequest :datatype-) "robot_planning/CartesianInterpolationRequest")
(setf (get robot_planning::CartesianInterpolationRequest :definition-)
      "int32 branch_id
float64[] joint_angles
float64[] start_pose  # [x, y, z, qx, qy, qz, qw]
float64[] goal_pose   # [x, y, z, qx, qy, qz, qw]
float64 duration      # 运动时间(秒)
float64 frequency     # 控制频率(Hz)
---
float64[] joint_trajectory  # Flattened array of joint configurations
bool success
string message 
")

(setf (get robot_planning::CartesianInterpolationResponse :md5sum-) "704fa7f95d1d0a395781e62e40e60c7e")
(setf (get robot_planning::CartesianInterpolationResponse :datatype-) "robot_planning/CartesianInterpolationResponse")
(setf (get robot_planning::CartesianInterpolationResponse :definition-)
      "int32 branch_id
float64[] joint_angles
float64[] start_pose  # [x, y, z, qx, qy, qz, qw]
float64[] goal_pose   # [x, y, z, qx, qy, qz, qw]
float64 duration      # 运动时间(秒)
float64 frequency     # 控制频率(Hz)
---
float64[] joint_trajectory  # Flattened array of joint configurations
bool success
string message 
")



(provide :robot_planning/CartesianInterpolation "704fa7f95d1d0a395781e62e40e60c7e")


