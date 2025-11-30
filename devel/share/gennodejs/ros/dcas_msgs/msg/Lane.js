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

class Lane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.lane_lines = null;
      this.lane_type = null;
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
      if (initObj.hasOwnProperty('lane_lines')) {
        this.lane_lines = initObj.lane_lines
      }
      else {
        this.lane_lines = [];
      }
      if (initObj.hasOwnProperty('lane_type')) {
        this.lane_type = initObj.lane_type
      }
      else {
        this.lane_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lane
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [lane_lines]
    // Serialize the length for message field [lane_lines]
    bufferOffset = _serializer.uint32(obj.lane_lines.length, buffer, bufferOffset);
    obj.lane_lines.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [lane_type]
    bufferOffset = _serializer.uint8(obj.lane_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lane
    let len;
    let data = new Lane(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [lane_lines]
    // Deserialize array length for message field [lane_lines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.lane_lines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lane_lines[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [lane_type]
    data.lane_type = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 12 * object.lane_lines.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dcas_msgs/Lane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7404a3f5e4db4c5f96339c55f4901f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint32 id
    # For simplicity use 2D points in map frame
    geometry_msgs/Point32[] lane_lines
    
    # Lane type definitions
    uint8 TYPE_UNKNOWN=0
    uint8 TYPE_WHITE_SOLID=1
    uint8 TYPE_WHITE_DASHED=2
    uint8 TYPE_YELLOW_SOLID=3
    uint8 TYPE_YELLOW_DASHED=4
    uint8 TYPE_DOUBLE_YELLOW_SOLID=5
    uint8 lane_type
    
    
    
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
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lane(null);
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

    if (msg.lane_lines !== undefined) {
      resolved.lane_lines = new Array(msg.lane_lines.length);
      for (let i = 0; i < resolved.lane_lines.length; ++i) {
        resolved.lane_lines[i] = geometry_msgs.msg.Point32.Resolve(msg.lane_lines[i]);
      }
    }
    else {
      resolved.lane_lines = []
    }

    if (msg.lane_type !== undefined) {
      resolved.lane_type = msg.lane_type;
    }
    else {
      resolved.lane_type = 0
    }

    return resolved;
    }
};

// Constants for message
Lane.Constants = {
  TYPE_UNKNOWN: 0,
  TYPE_WHITE_SOLID: 1,
  TYPE_WHITE_DASHED: 2,
  TYPE_YELLOW_SOLID: 3,
  TYPE_YELLOW_DASHED: 4,
  TYPE_DOUBLE_YELLOW_SOLID: 5,
}

module.exports = Lane;
