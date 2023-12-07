;; Auto-generated. Do not edit!


(when (boundp 'robot_follow_yolo::anchor_box)
  (if (not (find-package "ROBOT_FOLLOW_YOLO"))
    (make-package "ROBOT_FOLLOW_YOLO"))
  (shadow 'anchor_box (find-package "ROBOT_FOLLOW_YOLO")))
(unless (find-package "ROBOT_FOLLOW_YOLO::ANCHOR_BOX")
  (make-package "ROBOT_FOLLOW_YOLO::ANCHOR_BOX"))

(in-package "ROS")
;;//! \htmlinclude anchor_box.msg.html


(defclass robot_follow_yolo::anchor_box
  :super ros::object
  :slots (_x1 _y1 _x2 _y2 _proportion _classes ))

(defmethod robot_follow_yolo::anchor_box
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:y1 __y1) 0.0)
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    ((:proportion __proportion) 0.0)
    ((:classes __classes) 0.0)
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _y1 (float __y1))
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   (setq _proportion (float __proportion))
   (setq _classes (float __classes))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:proportion
   (&optional __proportion)
   (if __proportion (setq _proportion __proportion)) _proportion)
  (:classes
   (&optional __classes)
   (if __classes (setq _classes __classes)) _classes)
  (:serialization-length
   ()
   (+
    ;; float32 _x1
    4
    ;; float32 _y1
    4
    ;; float32 _x2
    4
    ;; float32 _y2
    4
    ;; float32 _proportion
    4
    ;; float32 _classes
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x1
       (sys::poke _x1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y1
       (sys::poke _y1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y2
       (sys::poke _y2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _proportion
       (sys::poke _proportion (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _classes
       (sys::poke _classes (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x1
     (setq _x1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y1
     (setq _y1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x2
     (setq _x2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y2
     (setq _y2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _proportion
     (setq _proportion (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _classes
     (setq _classes (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_follow_yolo::anchor_box :md5sum-) "961bea4c7f3887f07068935c6ef2d06e")
(setf (get robot_follow_yolo::anchor_box :datatype-) "robot_follow_yolo/anchor_box")
(setf (get robot_follow_yolo::anchor_box :definition-)
      "float32 x1
float32 y1
float32 x2
float32 y2
float32 proportion
float32 classes



")



(provide :robot_follow_yolo/anchor_box "961bea4c7f3887f07068935c6ef2d06e")


