; Auto-generated. Do not edit!


(cl:in-package cs_merge_msgs-srv)


;//! \htmlinclude getWorld-request.msg.html

(cl:defclass <getWorld-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getWorld-request (<getWorld-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getWorld-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getWorld-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cs_merge_msgs-srv:<getWorld-request> is deprecated: use cs_merge_msgs-srv:getWorld-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getWorld-request>) ostream)
  "Serializes a message object of type '<getWorld-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getWorld-request>) istream)
  "Deserializes a message object of type '<getWorld-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getWorld-request>)))
  "Returns string type for a service object of type '<getWorld-request>"
  "cs_merge_msgs/getWorldRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getWorld-request)))
  "Returns string type for a service object of type 'getWorld-request"
  "cs_merge_msgs/getWorldRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getWorld-request>)))
  "Returns md5sum for a message object of type '<getWorld-request>"
  "a57cc86655fe712508fa4becba72db19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getWorld-request)))
  "Returns md5sum for a message object of type 'getWorld-request"
  "a57cc86655fe712508fa4becba72db19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getWorld-request>)))
  "Returns full string definition for message of type '<getWorld-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getWorld-request)))
  "Returns full string definition for message of type 'getWorld-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getWorld-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getWorld-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getWorld-request
))
;//! \htmlinclude getWorld-response.msg.html

(cl:defclass <getWorld-response> (roslisp-msg-protocol:ros-message)
  ((world
    :reader world
    :initarg :world
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid)))
)

(cl:defclass getWorld-response (<getWorld-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getWorld-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getWorld-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cs_merge_msgs-srv:<getWorld-response> is deprecated: use cs_merge_msgs-srv:getWorld-response instead.")))

(cl:ensure-generic-function 'world-val :lambda-list '(m))
(cl:defmethod world-val ((m <getWorld-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cs_merge_msgs-srv:world-val is deprecated.  Use cs_merge_msgs-srv:world instead.")
  (world m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getWorld-response>) ostream)
  "Serializes a message object of type '<getWorld-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'world) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getWorld-response>) istream)
  "Deserializes a message object of type '<getWorld-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'world) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getWorld-response>)))
  "Returns string type for a service object of type '<getWorld-response>"
  "cs_merge_msgs/getWorldResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getWorld-response)))
  "Returns string type for a service object of type 'getWorld-response"
  "cs_merge_msgs/getWorldResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getWorld-response>)))
  "Returns md5sum for a message object of type '<getWorld-response>"
  "a57cc86655fe712508fa4becba72db19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getWorld-response)))
  "Returns md5sum for a message object of type 'getWorld-response"
  "a57cc86655fe712508fa4becba72db19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getWorld-response>)))
  "Returns full string definition for message of type '<getWorld-response>"
  (cl:format cl:nil "~%nav_msgs/OccupancyGrid world~%~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getWorld-response)))
  "Returns full string definition for message of type 'getWorld-response"
  (cl:format cl:nil "~%nav_msgs/OccupancyGrid world~%~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getWorld-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'world))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getWorld-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getWorld-response
    (cl:cons ':world (world msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getWorld)))
  'getWorld-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getWorld)))
  'getWorld-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getWorld)))
  "Returns string type for a service object of type '<getWorld>"
  "cs_merge_msgs/getWorld")