; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude GetBaseLinkPose-request.msg.html

(cl:defclass <GetBaseLinkPose-request> (roslisp-msg-protocol:ros-message)
  ((joint_angles_branch1
    :reader joint_angles_branch1
    :initarg :joint_angles_branch1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (joint_angles_branch4
    :reader joint_angles_branch4
    :initarg :joint_angles_branch4
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetBaseLinkPose-request (<GetBaseLinkPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBaseLinkPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBaseLinkPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<GetBaseLinkPose-request> is deprecated: use robot_control-srv:GetBaseLinkPose-request instead.")))

(cl:ensure-generic-function 'joint_angles_branch1-val :lambda-list '(m))
(cl:defmethod joint_angles_branch1-val ((m <GetBaseLinkPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:joint_angles_branch1-val is deprecated.  Use robot_control-srv:joint_angles_branch1 instead.")
  (joint_angles_branch1 m))

(cl:ensure-generic-function 'joint_angles_branch4-val :lambda-list '(m))
(cl:defmethod joint_angles_branch4-val ((m <GetBaseLinkPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:joint_angles_branch4-val is deprecated.  Use robot_control-srv:joint_angles_branch4 instead.")
  (joint_angles_branch4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBaseLinkPose-request>) ostream)
  "Serializes a message object of type '<GetBaseLinkPose-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_angles_branch1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_angles_branch1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_angles_branch4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_angles_branch4))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBaseLinkPose-request>) istream)
  "Deserializes a message object of type '<GetBaseLinkPose-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_angles_branch1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_angles_branch1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_angles_branch4) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_angles_branch4)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBaseLinkPose-request>)))
  "Returns string type for a service object of type '<GetBaseLinkPose-request>"
  "robot_control/GetBaseLinkPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBaseLinkPose-request)))
  "Returns string type for a service object of type 'GetBaseLinkPose-request"
  "robot_control/GetBaseLinkPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBaseLinkPose-request>)))
  "Returns md5sum for a message object of type '<GetBaseLinkPose-request>"
  "124f2f1fb4bf270787e14bd14a8d0a4b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBaseLinkPose-request)))
  "Returns md5sum for a message object of type 'GetBaseLinkPose-request"
  "124f2f1fb4bf270787e14bd14a8d0a4b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBaseLinkPose-request>)))
  "Returns full string definition for message of type '<GetBaseLinkPose-request>"
  (cl:format cl:nil "float64[] joint_angles_branch1~%float64[] joint_angles_branch4~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBaseLinkPose-request)))
  "Returns full string definition for message of type 'GetBaseLinkPose-request"
  (cl:format cl:nil "float64[] joint_angles_branch1~%float64[] joint_angles_branch4~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBaseLinkPose-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_angles_branch1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_angles_branch4) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBaseLinkPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBaseLinkPose-request
    (cl:cons ':joint_angles_branch1 (joint_angles_branch1 msg))
    (cl:cons ':joint_angles_branch4 (joint_angles_branch4 msg))
))
;//! \htmlinclude GetBaseLinkPose-response.msg.html

(cl:defclass <GetBaseLinkPose-response> (roslisp-msg-protocol:ros-message)
  ((base_link_pose
    :reader base_link_pose
    :initarg :base_link_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GetBaseLinkPose-response (<GetBaseLinkPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBaseLinkPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBaseLinkPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<GetBaseLinkPose-response> is deprecated: use robot_control-srv:GetBaseLinkPose-response instead.")))

(cl:ensure-generic-function 'base_link_pose-val :lambda-list '(m))
(cl:defmethod base_link_pose-val ((m <GetBaseLinkPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:base_link_pose-val is deprecated.  Use robot_control-srv:base_link_pose instead.")
  (base_link_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBaseLinkPose-response>) ostream)
  "Serializes a message object of type '<GetBaseLinkPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base_link_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBaseLinkPose-response>) istream)
  "Deserializes a message object of type '<GetBaseLinkPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base_link_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBaseLinkPose-response>)))
  "Returns string type for a service object of type '<GetBaseLinkPose-response>"
  "robot_control/GetBaseLinkPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBaseLinkPose-response)))
  "Returns string type for a service object of type 'GetBaseLinkPose-response"
  "robot_control/GetBaseLinkPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBaseLinkPose-response>)))
  "Returns md5sum for a message object of type '<GetBaseLinkPose-response>"
  "124f2f1fb4bf270787e14bd14a8d0a4b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBaseLinkPose-response)))
  "Returns md5sum for a message object of type 'GetBaseLinkPose-response"
  "124f2f1fb4bf270787e14bd14a8d0a4b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBaseLinkPose-response>)))
  "Returns full string definition for message of type '<GetBaseLinkPose-response>"
  (cl:format cl:nil "# Response: 位姿~%geometry_msgs/Pose base_link_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBaseLinkPose-response)))
  "Returns full string definition for message of type 'GetBaseLinkPose-response"
  (cl:format cl:nil "# Response: 位姿~%geometry_msgs/Pose base_link_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBaseLinkPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base_link_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBaseLinkPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBaseLinkPose-response
    (cl:cons ':base_link_pose (base_link_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetBaseLinkPose)))
  'GetBaseLinkPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetBaseLinkPose)))
  'GetBaseLinkPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBaseLinkPose)))
  "Returns string type for a service object of type '<GetBaseLinkPose>"
  "robot_control/GetBaseLinkPose")