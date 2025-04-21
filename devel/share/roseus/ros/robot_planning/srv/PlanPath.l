;; Auto-generated. Do not edit!


(when (boundp 'robot_planning::PlanPath)
  (if (not (find-package "ROBOT_PLANNING"))
    (make-package "ROBOT_PLANNING"))
  (shadow 'PlanPath (find-package "ROBOT_PLANNING")))
(unless (find-package "ROBOT_PLANNING::PLANPATH")
  (make-package "ROBOT_PLANNING::PLANPATH"))
(unless (find-package "ROBOT_PLANNING::PLANPATHREQUEST")
  (make-package "ROBOT_PLANNING::PLANPATHREQUEST"))
(unless (find-package "ROBOT_PLANNING::PLANPATHRESPONSE")
  (make-package "ROBOT_PLANNING::PLANPATHRESPONSE"))

(in-package "ROS")





(defclass robot_planning::PlanPathRequest
  :super ros::object
  :slots ())

(defmethod robot_planning::PlanPathRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass robot_planning::PlanPathResponse
  :super ros::object
  :slots (_success _message ))

(defmethod robot_planning::PlanPathResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot_planning::PlanPath
  :super ros::object
  :slots ())

(setf (get robot_planning::PlanPath :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get robot_planning::PlanPath :datatype-) "robot_planning/PlanPath")
(setf (get robot_planning::PlanPath :request) robot_planning::PlanPathRequest)
(setf (get robot_planning::PlanPath :response) robot_planning::PlanPathResponse)

(defmethod robot_planning::PlanPathRequest
  (:response () (instance robot_planning::PlanPathResponse :init)))

(setf (get robot_planning::PlanPathRequest :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get robot_planning::PlanPathRequest :datatype-) "robot_planning/PlanPathRequest")
(setf (get robot_planning::PlanPathRequest :definition-)
      "# 空请求，触发规划
---
bool success
string message

")

(setf (get robot_planning::PlanPathResponse :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get robot_planning::PlanPathResponse :datatype-) "robot_planning/PlanPathResponse")
(setf (get robot_planning::PlanPathResponse :definition-)
      "# 空请求，触发规划
---
bool success
string message

")



(provide :robot_planning/PlanPath "937c9679a518e3a18d831e57125ea522")


