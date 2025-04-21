; Auto-generated. Do not edit!


(cl:in-package robot_planning-srv)


;//! \htmlinclude PlanPath-request.msg.html

(cl:defclass <PlanPath-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PlanPath-request (<PlanPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_planning-srv:<PlanPath-request> is deprecated: use robot_planning-srv:PlanPath-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanPath-request>) ostream)
  "Serializes a message object of type '<PlanPath-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanPath-request>) istream)
  "Deserializes a message object of type '<PlanPath-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanPath-request>)))
  "Returns string type for a service object of type '<PlanPath-request>"
  "robot_planning/PlanPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanPath-request)))
  "Returns string type for a service object of type 'PlanPath-request"
  "robot_planning/PlanPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanPath-request>)))
  "Returns md5sum for a message object of type '<PlanPath-request>"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanPath-request)))
  "Returns md5sum for a message object of type 'PlanPath-request"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanPath-request>)))
  "Returns full string definition for message of type '<PlanPath-request>"
  (cl:format cl:nil "# 空请求，触发规划~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanPath-request)))
  "Returns full string definition for message of type 'PlanPath-request"
  (cl:format cl:nil "# 空请求，触发规划~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanPath-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanPath-request
))
;//! \htmlinclude PlanPath-response.msg.html

(cl:defclass <PlanPath-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass PlanPath-response (<PlanPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_planning-srv:<PlanPath-response> is deprecated: use robot_planning-srv:PlanPath-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlanPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_planning-srv:success-val is deprecated.  Use robot_planning-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PlanPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_planning-srv:message-val is deprecated.  Use robot_planning-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanPath-response>) ostream)
  "Serializes a message object of type '<PlanPath-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanPath-response>) istream)
  "Deserializes a message object of type '<PlanPath-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanPath-response>)))
  "Returns string type for a service object of type '<PlanPath-response>"
  "robot_planning/PlanPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanPath-response)))
  "Returns string type for a service object of type 'PlanPath-response"
  "robot_planning/PlanPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanPath-response>)))
  "Returns md5sum for a message object of type '<PlanPath-response>"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanPath-response)))
  "Returns md5sum for a message object of type 'PlanPath-response"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanPath-response>)))
  "Returns full string definition for message of type '<PlanPath-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanPath-response)))
  "Returns full string definition for message of type 'PlanPath-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanPath-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanPath-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanPath)))
  'PlanPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanPath)))
  'PlanPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanPath)))
  "Returns string type for a service object of type '<PlanPath>"
  "robot_planning/PlanPath")