;; Auto-generated. Do not edit!


(when (boundp 'barc::camera_out)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'camera_out (find-package "BARC")))
(unless (find-package "BARC::CAMERA_OUT")
  (make-package "BARC::CAMERA_OUT"))

(in-package "ROS")
;;//! \htmlinclude camera_out.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass barc::camera_out
  :super ros::object
  :slots (_s _curv_s _ey ))

(defmethod barc::camera_out
  (:init
   (&key
    ((:s __s) ())
    ((:curv_s __curv_s) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ey __ey) 0.0)
    )
   (send-super :init)
   (setq _s __s)
   (setq _curv_s __curv_s)
   (setq _ey (float __ey))
   self)
  (:s
   (&rest __s)
   (if (keywordp (car __s))
       (send* _s __s)
     (progn
       (if __s (setq _s (car __s)))
       _s)))
  (:curv_s
   (&optional __curv_s)
   (if __curv_s (setq _curv_s __curv_s)) _curv_s)
  (:ey
   (&optional __ey)
   (if __ey (setq _ey __ey)) _ey)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _s
    (apply #'+ (send-all _s :serialization-length)) 4
    ;; float64[] _curv_s
    (* 8    (length _curv_s)) 4
    ;; float64 _ey
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _s
     (write-long (length _s) s)
     (dolist (elem _s)
       (send elem :serialize s)
       )
     ;; float64[] _curv_s
     (write-long (length _curv_s) s)
     (dotimes (i (length _curv_s))
       (sys::poke (elt _curv_s i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _ey
       (sys::poke _ey (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _s
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _s (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _s)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _curv_s
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _curv_s (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _curv_s i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _ey
     (setq _ey (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get barc::camera_out :md5sum-) "20a2606aec8bfaba63eeafd610f117a5")
(setf (get barc::camera_out :datatype-) "barc/camera_out")
(setf (get barc::camera_out :definition-)
      "geometry_msgs/Point[] s
float64[] curv_s
float64 ey
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :barc/camera_out "20a2606aec8bfaba63eeafd610f117a5")


