
(cl:in-package :asdf)

(defsystem "robot_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GetBaseLinkPose" :depends-on ("_package_GetBaseLinkPose"))
    (:file "_package_GetBaseLinkPose" :depends-on ("_package"))
  ))