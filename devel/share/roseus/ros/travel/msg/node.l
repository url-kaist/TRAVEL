;; Auto-generated. Do not edit!


(when (boundp 'travel::node)
  (if (not (find-package "TRAVEL"))
    (make-package "TRAVEL"))
  (shadow 'node (find-package "TRAVEL")))
(unless (find-package "TRAVEL::NODE")
  (make-package "TRAVEL::NODE"))

(in-package "ROS")
;;//! \htmlinclude node.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass travel::node
  :super ros::object
  :slots (_idx _header _pose _lidar _ground_estimate ))

(defmethod travel::node
  (:init
   (&key
    ((:idx __idx) 0)
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:lidar __lidar) (instance sensor_msgs::PointCloud2 :init))
    ((:ground_estimate __ground_estimate) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _idx (round __idx))
   (setq _header __header)
   (setq _pose __pose)
   (setq _lidar __lidar)
   (setq _ground_estimate __ground_estimate)
   self)
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:lidar
   (&rest __lidar)
   (if (keywordp (car __lidar))
       (send* _lidar __lidar)
     (progn
       (if __lidar (setq _lidar (car __lidar)))
       _lidar)))
  (:ground_estimate
   (&rest __ground_estimate)
   (if (keywordp (car __ground_estimate))
       (send* _ground_estimate __ground_estimate)
     (progn
       (if __ground_estimate (setq _ground_estimate (car __ground_estimate)))
       _ground_estimate)))
  (:serialization-length
   ()
   (+
    ;; int32 _idx
    4
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; sensor_msgs/PointCloud2 _lidar
    (send _lidar :serialization-length)
    ;; sensor_msgs/PointCloud2 _ground_estimate
    (send _ground_estimate :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _idx
       (write-long _idx s)
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; sensor_msgs/PointCloud2 _lidar
       (send _lidar :serialize s)
     ;; sensor_msgs/PointCloud2 _ground_estimate
       (send _ground_estimate :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _idx
     (setq _idx (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; sensor_msgs/PointCloud2 _lidar
     (send _lidar :deserialize buf ptr-) (incf ptr- (send _lidar :serialization-length))
   ;; sensor_msgs/PointCloud2 _ground_estimate
     (send _ground_estimate :deserialize buf ptr-) (incf ptr- (send _ground_estimate :serialization-length))
   ;;
   self)
  )

(setf (get travel::node :md5sum-) "8ffdb3dcfd475161209f2ce2c04a5bcc")
(setf (get travel::node :datatype-) "travel/node")
(setf (get travel::node :definition-)
      "int32 idx
std_msgs/Header header
geometry_msgs/Pose pose
sensor_msgs/PointCloud2 lidar
sensor_msgs/PointCloud2 ground_estimate # estimate
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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

")



(provide :travel/node "8ffdb3dcfd475161209f2ce2c04a5bcc")


