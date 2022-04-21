; Auto-generated. Do not edit!


(cl:in-package travel-msg)


;//! \htmlinclude node.msg.html

(cl:defclass <node> (roslisp-msg-protocol:ros-message)
  ((idx
    :reader idx
    :initarg :idx
    :type cl:integer
    :initform 0)
   (header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (lidar
    :reader lidar
    :initarg :lidar
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (ground_estimate
    :reader ground_estimate
    :initarg :ground_estimate
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass node (<node>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <node>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'node)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name travel-msg:<node> is deprecated: use travel-msg:node instead.")))

(cl:ensure-generic-function 'idx-val :lambda-list '(m))
(cl:defmethod idx-val ((m <node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader travel-msg:idx-val is deprecated.  Use travel-msg:idx instead.")
  (idx m))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader travel-msg:header-val is deprecated.  Use travel-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader travel-msg:pose-val is deprecated.  Use travel-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'lidar-val :lambda-list '(m))
(cl:defmethod lidar-val ((m <node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader travel-msg:lidar-val is deprecated.  Use travel-msg:lidar instead.")
  (lidar m))

(cl:ensure-generic-function 'ground_estimate-val :lambda-list '(m))
(cl:defmethod ground_estimate-val ((m <node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader travel-msg:ground_estimate-val is deprecated.  Use travel-msg:ground_estimate instead.")
  (ground_estimate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <node>) ostream)
  "Serializes a message object of type '<node>"
  (cl:let* ((signed (cl:slot-value msg 'idx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lidar) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ground_estimate) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <node>) istream)
  "Deserializes a message object of type '<node>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lidar) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ground_estimate) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<node>)))
  "Returns string type for a message object of type '<node>"
  "travel/node")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'node)))
  "Returns string type for a message object of type 'node"
  "travel/node")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<node>)))
  "Returns md5sum for a message object of type '<node>"
  "8ffdb3dcfd475161209f2ce2c04a5bcc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'node)))
  "Returns md5sum for a message object of type 'node"
  "8ffdb3dcfd475161209f2ce2c04a5bcc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<node>)))
  "Returns full string definition for message of type '<node>"
  (cl:format cl:nil "int32 idx~%std_msgs/Header header~%geometry_msgs/Pose pose~%sensor_msgs/PointCloud2 lidar~%sensor_msgs/PointCloud2 ground_estimate # estimate~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'node)))
  "Returns full string definition for message of type 'node"
  (cl:format cl:nil "int32 idx~%std_msgs/Header header~%geometry_msgs/Pose pose~%sensor_msgs/PointCloud2 lidar~%sensor_msgs/PointCloud2 ground_estimate # estimate~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <node>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lidar))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ground_estimate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <node>))
  "Converts a ROS message object to a list"
  (cl:list 'node
    (cl:cons ':idx (idx msg))
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':lidar (lidar msg))
    (cl:cons ':ground_estimate (ground_estimate msg))
))
