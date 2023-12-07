
(cl:in-package :asdf)

(defsystem "robot_follow_yolo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "anchor_box" :depends-on ("_package_anchor_box"))
    (:file "_package_anchor_box" :depends-on ("_package"))
  ))