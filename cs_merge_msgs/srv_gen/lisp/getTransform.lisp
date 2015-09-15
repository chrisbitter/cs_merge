; Auto-generated. Do not edit!


(cl:in-package cs_merge_msgs-srv)


;//! \htmlinclude getTransform-request.msg.html

(cl:defclass <getTransform-request> (roslisp-msg-protocol:ros-message)
  ((topic_map_one
    :reader topic_map_one
    :initarg :topic_map_one
    :type cl:string
    :initform "")
   (topic_map_two
    :reader topic_map_two
    :initarg :topic_map_two
    :type cl:string
    :initform ""))
)

(cl:defclass getTransform-request (<getTransform-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getTransform-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getTransform-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cs_merge_msgs-srv:<getTransform-request> is deprecated: use cs_merge_msgs-srv:getTransform-request instead.")))

(cl:ensure-generic-function 'topic_map_one-val :lambda-list '(m))
(cl:defmethod topic_map_one-val ((m <getTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cs_merge_msgs-srv:topic_map_one-val is deprecated.  Use cs_merge_msgs-srv:topic_map_one instead.")
  (topic_map_one m))

(cl:ensure-generic-function 'topic_map_two-val :lambda-list '(m))
(cl:defmethod topic_map_two-val ((m <getTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cs_merge_msgs-srv:topic_map_two-val is deprecated.  Use cs_merge_msgs-srv:topic_map_two instead.")
  (topic_map_two m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getTransform-request>) ostream)
  "Serializes a message object of type '<getTransform-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic_map_one))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic_map_one))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic_map_two))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic_map_two))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getTransform-request>) istream)
  "Deserializes a message object of type '<getTransform-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic_map_one) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic_map_one) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic_map_two) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic_map_two) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getTransform-request>)))
  "Returns string type for a service object of type '<getTransform-request>"
  "cs_merge_msgs/getTransformRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTransform-request)))
  "Returns string type for a service object of type 'getTransform-request"
  "cs_merge_msgs/getTransformRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getTransform-request>)))
  "Returns md5sum for a message object of type '<getTransform-request>"
  "972de1e3b8810d34e2aa284314fffb13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getTransform-request)))
  "Returns md5sum for a message object of type 'getTransform-request"
  "972de1e3b8810d34e2aa284314fffb13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getTransform-request>)))
  "Returns full string definition for message of type '<getTransform-request>"
  (cl:format cl:nil "~%string topic_map_one~%string topic_map_two~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getTransform-request)))
  "Returns full string definition for message of type 'getTransform-request"
  (cl:format cl:nil "~%string topic_map_one~%string topic_map_two~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getTransform-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'topic_map_one))
     4 (cl:length (cl:slot-value msg 'topic_map_two))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getTransform-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getTransform-request
    (cl:cons ':topic_map_one (topic_map_one msg))
    (cl:cons ':topic_map_two (topic_map_two msg))
))
;//! \htmlinclude getTransform-response.msg.html

(cl:defclass <getTransform-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cs_merge_msgs-msg:transform
    :initform (cl:make-instance 'cs_merge_msgs-msg:transform)))
)

(cl:defclass getTransform-response (<getTransform-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getTransform-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getTransform-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cs_merge_msgs-srv:<getTransform-response> is deprecated: use cs_merge_msgs-srv:getTransform-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <getTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cs_merge_msgs-srv:result-val is deprecated.  Use cs_merge_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getTransform-response>) ostream)
  "Serializes a message object of type '<getTransform-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getTransform-response>) istream)
  "Deserializes a message object of type '<getTransform-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getTransform-response>)))
  "Returns string type for a service object of type '<getTransform-response>"
  "cs_merge_msgs/getTransformResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTransform-response)))
  "Returns string type for a service object of type 'getTransform-response"
  "cs_merge_msgs/getTransformResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getTransform-response>)))
  "Returns md5sum for a message object of type '<getTransform-response>"
  "972de1e3b8810d34e2aa284314fffb13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getTransform-response)))
  "Returns md5sum for a message object of type 'getTransform-response"
  "972de1e3b8810d34e2aa284314fffb13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getTransform-response>)))
  "Returns full string definition for message of type '<getTransform-response>"
  (cl:format cl:nil "~%cs_merge_msgs/transform result~%~%~%================================================================================~%MSG: cs_merge_msgs/transform~%time stamp~%float64 rotation~%float64 dx~%float64 dy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getTransform-response)))
  "Returns full string definition for message of type 'getTransform-response"
  (cl:format cl:nil "~%cs_merge_msgs/transform result~%~%~%================================================================================~%MSG: cs_merge_msgs/transform~%time stamp~%float64 rotation~%float64 dx~%float64 dy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getTransform-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getTransform-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getTransform-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getTransform)))
  'getTransform-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getTransform)))
  'getTransform-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTransform)))
  "Returns string type for a service object of type '<getTransform>"
  "cs_merge_msgs/getTransform")