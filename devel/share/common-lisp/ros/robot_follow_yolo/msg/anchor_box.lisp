; Auto-generated. Do not edit!


(cl:in-package robot_follow_yolo-msg)


;//! \htmlinclude anchor_box.msg.html

(cl:defclass <anchor_box> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type cl:float
    :initform 0.0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:float
    :initform 0.0)
   (x2
    :reader x2
    :initarg :x2
    :type cl:float
    :initform 0.0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:float
    :initform 0.0)
   (proportion
    :reader proportion
    :initarg :proportion
    :type cl:float
    :initform 0.0)
   (classes
    :reader classes
    :initarg :classes
    :type cl:float
    :initform 0.0))
)

(cl:defclass anchor_box (<anchor_box>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <anchor_box>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'anchor_box)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_follow_yolo-msg:<anchor_box> is deprecated: use robot_follow_yolo-msg:anchor_box instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <anchor_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_follow_yolo-msg:x1-val is deprecated.  Use robot_follow_yolo-msg:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <anchor_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_follow_yolo-msg:y1-val is deprecated.  Use robot_follow_yolo-msg:y1 instead.")
  (y1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <anchor_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_follow_yolo-msg:x2-val is deprecated.  Use robot_follow_yolo-msg:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <anchor_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_follow_yolo-msg:y2-val is deprecated.  Use robot_follow_yolo-msg:y2 instead.")
  (y2 m))

(cl:ensure-generic-function 'proportion-val :lambda-list '(m))
(cl:defmethod proportion-val ((m <anchor_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_follow_yolo-msg:proportion-val is deprecated.  Use robot_follow_yolo-msg:proportion instead.")
  (proportion m))

(cl:ensure-generic-function 'classes-val :lambda-list '(m))
(cl:defmethod classes-val ((m <anchor_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_follow_yolo-msg:classes-val is deprecated.  Use robot_follow_yolo-msg:classes instead.")
  (classes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <anchor_box>) ostream)
  "Serializes a message object of type '<anchor_box>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'proportion))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'classes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <anchor_box>) istream)
  "Deserializes a message object of type '<anchor_box>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'proportion) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'classes) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<anchor_box>)))
  "Returns string type for a message object of type '<anchor_box>"
  "robot_follow_yolo/anchor_box")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'anchor_box)))
  "Returns string type for a message object of type 'anchor_box"
  "robot_follow_yolo/anchor_box")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<anchor_box>)))
  "Returns md5sum for a message object of type '<anchor_box>"
  "961bea4c7f3887f07068935c6ef2d06e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'anchor_box)))
  "Returns md5sum for a message object of type 'anchor_box"
  "961bea4c7f3887f07068935c6ef2d06e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<anchor_box>)))
  "Returns full string definition for message of type '<anchor_box>"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 proportion~%float32 classes~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'anchor_box)))
  "Returns full string definition for message of type 'anchor_box"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 proportion~%float32 classes~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <anchor_box>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <anchor_box>))
  "Converts a ROS message object to a list"
  (cl:list 'anchor_box
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':y1 (y1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':y2 (y2 msg))
    (cl:cons ':proportion (proportion msg))
    (cl:cons ':classes (classes msg))
))
