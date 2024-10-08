;; Auto-generated. Do not edit!


(when (boundp 'gpd_grasp_msgs::GraspConfigList)
  (if (not (find-package "GPD_GRASP_MSGS"))
    (make-package "GPD_GRASP_MSGS"))
  (shadow 'GraspConfigList (find-package "GPD_GRASP_MSGS")))
(unless (find-package "GPD_GRASP_MSGS::GRASPCONFIGLIST")
  (make-package "GPD_GRASP_MSGS::GRASPCONFIGLIST"))

(in-package "ROS")
;;//! \htmlinclude GraspConfigList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gpd_grasp_msgs::GraspConfigList
  :super ros::object
  :slots (_header _grasps ))

(defmethod gpd_grasp_msgs::GraspConfigList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:grasps __grasps) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _grasps __grasps)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:grasps
   (&rest __grasps)
   (if (keywordp (car __grasps))
       (send* _grasps __grasps)
     (progn
       (if __grasps (setq _grasps (car __grasps)))
       _grasps)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; gpd_grasp_msgs/GraspConfig[] _grasps
    (apply #'+ (send-all _grasps :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; gpd_grasp_msgs/GraspConfig[] _grasps
     (write-long (length _grasps) s)
     (dolist (elem _grasps)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; gpd_grasp_msgs/GraspConfig[] _grasps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grasps (let (r) (dotimes (i n) (push (instance gpd_grasp_msgs::GraspConfig :init) r)) r))
     (dolist (elem- _grasps)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get gpd_grasp_msgs::GraspConfigList :md5sum-) "5e1675cb2ef4eacde35945da8d7b8c6f")
(setf (get gpd_grasp_msgs::GraspConfigList :datatype-) "gpd_grasp_msgs/GraspConfigList")
(setf (get gpd_grasp_msgs::GraspConfigList :definition-)
      "# This message stores a list of grasp configurations.

# The time of acquisition, and the coordinate frame ID.
Header header

# The list of grasp configurations.
gpd_grasp_msgs/GraspConfig[] grasps

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: gpd_grasp_msgs/GraspConfig
# This message describes a 2-finger grasp configuration by its 6-DOF pose, 
# consisting of a 3-DOF position and 3-DOF orientation, and the opening 
# width of the robot hand.

# Position
geometry_msgs/Point bottom # centered bottom/base of the robot hand)
geometry_msgs/Point top # centered top/fingertip of the robot hand)
geometry_msgs/Point surface # centered position on object surface

# Orientation represented as three axis (R = [approach binormal axis])
geometry_msgs/Vector3 approach # The grasp approach direction
geometry_msgs/Vector3 binormal # The binormal
geometry_msgs/Vector3 axis # The hand axis

geometry_msgs/Point sample # Point at which the grasp was found

std_msgs/Float32 width # Required aperture (opening width) of the robot hand 

std_msgs/Float32 score # Score assigned to the grasp by the classifier

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :gpd_grasp_msgs/GraspConfigList "5e1675cb2ef4eacde35945da8d7b8c6f")


