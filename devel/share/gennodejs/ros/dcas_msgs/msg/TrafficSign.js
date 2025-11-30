// Auto-generated. Do not edit!

// (in-package dcas_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrafficSign {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.pose = null;
      this.sign_class = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('sign_class')) {
        this.sign_class = initObj.sign_class
      }
      else {
        this.sign_class = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrafficSign
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [sign_class]
    bufferOffset = _serializer.uint8(obj.sign_class, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrafficSign
    let len;
    let data = new TrafficSign(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [sign_class]
    data.sign_class = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 61;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dcas_msgs/TrafficSign';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c95b6f353f8d2898545192a2a0de59a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint32 id
    geometry_msgs/Pose pose
    uint8 sign_class
    
    # Traffic sign classes
    uint8 CLASS_UNKNOWN=0
    uint8 CLASS_STOP=1
    uint8 CLASS_YIELD=2
    uint8 CLASS_SPEED_LIMIT_30=3
    uint8 CLASS_SPEED_LIMIT_50=4
    uint8 CLASS_SPEED_LIMIT_60=5
    uint8 CLASS_SPEED_LIMIT_80=6
    uint8 CLASS_NO_ENTRY=7
    uint8 CLASS_NO_PARKING=8
    uint8 CLASS_PEDESTRIAN_CROSSING=9
    uint8 CLASS_SCHOOL_ZONE=10
    uint8 CLASS_CONSTRUCTION=11
    uint8 CLASS_TURN_LEFT=12
    uint8 CLASS_TURN_RIGHT=13
    uint8 CLASS_GO_STRAIGHT=14
    
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrafficSign(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.sign_class !== undefined) {
      resolved.sign_class = msg.sign_class;
    }
    else {
      resolved.sign_class = 0
    }

    return resolved;
    }
};

// Constants for message
TrafficSign.Constants = {
  CLASS_UNKNOWN: 0,
  CLASS_STOP: 1,
  CLASS_YIELD: 2,
  CLASS_SPEED_LIMIT_30: 3,
  CLASS_SPEED_LIMIT_50: 4,
  CLASS_SPEED_LIMIT_60: 5,
  CLASS_SPEED_LIMIT_80: 6,
  CLASS_NO_ENTRY: 7,
  CLASS_NO_PARKING: 8,
  CLASS_PEDESTRIAN_CROSSING: 9,
  CLASS_SCHOOL_ZONE: 10,
  CLASS_CONSTRUCTION: 11,
  CLASS_TURN_LEFT: 12,
  CLASS_TURN_RIGHT: 13,
  CLASS_GO_STRAIGHT: 14,
}

module.exports = TrafficSign;
