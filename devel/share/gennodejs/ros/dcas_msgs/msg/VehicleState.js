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

class VehicleState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose = null;
      this.twist = null;
      this.acceleration = null;
      this.steering_wheel_angle_deg = null;
      this.wheel_speeds_mps = null;
      this.accelerator_pedal = null;
      this.brake_pedal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('steering_wheel_angle_deg')) {
        this.steering_wheel_angle_deg = initObj.steering_wheel_angle_deg
      }
      else {
        this.steering_wheel_angle_deg = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_speeds_mps')) {
        this.wheel_speeds_mps = initObj.wheel_speeds_mps
      }
      else {
        this.wheel_speeds_mps = [];
      }
      if (initObj.hasOwnProperty('accelerator_pedal')) {
        this.accelerator_pedal = initObj.accelerator_pedal
      }
      else {
        this.accelerator_pedal = 0.0;
      }
      if (initObj.hasOwnProperty('brake_pedal')) {
        this.brake_pedal = initObj.brake_pedal
      }
      else {
        this.brake_pedal = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [steering_wheel_angle_deg]
    bufferOffset = _serializer.float32(obj.steering_wheel_angle_deg, buffer, bufferOffset);
    // Serialize message field [wheel_speeds_mps]
    bufferOffset = _arraySerializer.float32(obj.wheel_speeds_mps, buffer, bufferOffset, null);
    // Serialize message field [accelerator_pedal]
    bufferOffset = _serializer.float32(obj.accelerator_pedal, buffer, bufferOffset);
    // Serialize message field [brake_pedal]
    bufferOffset = _serializer.float32(obj.brake_pedal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleState
    let len;
    let data = new VehicleState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [steering_wheel_angle_deg]
    data.steering_wheel_angle_deg = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wheel_speeds_mps]
    data.wheel_speeds_mps = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [accelerator_pedal]
    data.accelerator_pedal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [brake_pedal]
    data.brake_pedal = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.wheel_speeds_mps.length;
    return length + 144;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dcas_msgs/VehicleState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18b3a6879ecb65c8c32b4cb5c098ed3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    geometry_msgs/Pose pose
    geometry_msgs/Twist twist
    geometry_msgs/Vector3 acceleration
    float32 steering_wheel_angle_deg
    float32[] wheel_speeds_mps
    float32 accelerator_pedal
    float32 brake_pedal
    
    
    
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
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3 linear
    Vector3 angular
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VehicleState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.Twist.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.Twist()
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = geometry_msgs.msg.Vector3.Resolve(msg.acceleration)
    }
    else {
      resolved.acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.steering_wheel_angle_deg !== undefined) {
      resolved.steering_wheel_angle_deg = msg.steering_wheel_angle_deg;
    }
    else {
      resolved.steering_wheel_angle_deg = 0.0
    }

    if (msg.wheel_speeds_mps !== undefined) {
      resolved.wheel_speeds_mps = msg.wheel_speeds_mps;
    }
    else {
      resolved.wheel_speeds_mps = []
    }

    if (msg.accelerator_pedal !== undefined) {
      resolved.accelerator_pedal = msg.accelerator_pedal;
    }
    else {
      resolved.accelerator_pedal = 0.0
    }

    if (msg.brake_pedal !== undefined) {
      resolved.brake_pedal = msg.brake_pedal;
    }
    else {
      resolved.brake_pedal = 0.0
    }

    return resolved;
    }
};

module.exports = VehicleState;
