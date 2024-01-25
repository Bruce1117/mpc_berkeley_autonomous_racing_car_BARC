
(cl:in-package :asdf)

(defsystem "barc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
    (:file "camera_out" :depends-on ("_package_camera_out"))
    (:file "_package_camera_out" :depends-on ("_package"))
    (:file "camera_out" :depends-on ("_package_camera_out"))
    (:file "_package_camera_out" :depends-on ("_package"))
    (:file "input" :depends-on ("_package_input"))
    (:file "_package_input" :depends-on ("_package"))
    (:file "input" :depends-on ("_package_input"))
    (:file "_package_input" :depends-on ("_package"))
  ))