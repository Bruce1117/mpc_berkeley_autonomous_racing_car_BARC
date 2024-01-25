; Auto-generated. Do not edit!


(cl:in-package barc-msg)


;//! \htmlinclude camera_out.msg.html

(cl:defclass <camera_out> (roslisp-msg-protocol:ros-message)
  ((s
    :reader s
    :initarg :s
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (curv_s
    :reader curv_s
    :initarg :curv_s
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (ey
    :reader ey
    :initarg :ey
    :type cl:float
    :initform 0.0))
)

(cl:defclass camera_out (<camera_out>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <camera_out>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'camera_out)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name barc-msg:<camera_out> is deprecated: use barc-msg:camera_out instead.")))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <camera_out>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:s-val is deprecated.  Use barc-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'curv_s-val :lambda-list '(m))
(cl:defmethod curv_s-val ((m <camera_out>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:curv_s-val is deprecated.  Use barc-msg:curv_s instead.")
  (curv_s m))

(cl:ensure-generic-function 'ey-val :lambda-list '(m))
(cl:defmethod ey-val ((m <camera_out>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:ey-val is deprecated.  Use barc-msg:ey instead.")
  (ey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <camera_out>) ostream)
  "Serializes a message object of type '<camera_out>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 's))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'curv_s))))
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
   (cl:slot-value msg 'curv_s))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <camera_out>) istream)
  "Deserializes a message object of type '<camera_out>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 's) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 's)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'curv_s) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'curv_s)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ey) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<camera_out>)))
  "Returns string type for a message object of type '<camera_out>"
  "barc/camera_out")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'camera_out)))
  "Returns string type for a message object of type 'camera_out"
  "barc/camera_out")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<camera_out>)))
  "Returns md5sum for a message object of type '<camera_out>"
  "20a2606aec8bfaba63eeafd610f117a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'camera_out)))
  "Returns md5sum for a message object of type 'camera_out"
  "20a2606aec8bfaba63eeafd610f117a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<camera_out>)))
  "Returns full string definition for message of type '<camera_out>"
  (cl:format cl:nil "geometry_msgs/Point[] s~%float64[] curv_s~%float64 ey~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'camera_out)))
  "Returns full string definition for message of type 'camera_out"
  (cl:format cl:nil "geometry_msgs/Point[] s~%float64[] curv_s~%float64 ey~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <camera_out>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 's) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'curv_s) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <camera_out>))
  "Converts a ROS message object to a list"
  (cl:list 'camera_out
    (cl:cons ':s (s msg))
    (cl:cons ':curv_s (curv_s msg))
    (cl:cons ':ey (ey msg))
))
