
(cl:in-package :asdf)

(defsystem "widomX-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "multi_cam" :depends-on ("_package_multi_cam"))
    (:file "_package_multi_cam" :depends-on ("_package"))
  ))