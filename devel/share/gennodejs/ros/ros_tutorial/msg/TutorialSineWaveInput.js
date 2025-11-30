// Auto-generated. Do not edit!

// (in-package ros_tutorial.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TutorialSineWaveInput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.period = null;
      this.noise_standard_deviation = null;
      this.amplitude = null;
    }
    else {
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = 0.0;
      }
      if (initObj.hasOwnProperty('noise_standard_deviation')) {
        this.noise_standard_deviation = initObj.noise_standard_deviation
      }
      else {
        this.noise_standard_deviation = 0.0;
      }
      if (initObj.hasOwnProperty('amplitude')) {
        this.amplitude = initObj.amplitude
      }
      else {
        this.amplitude = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TutorialSineWaveInput
    // Serialize message field [period]
    bufferOffset = _serializer.float64(obj.period, buffer, bufferOffset);
    // Serialize message field [noise_standard_deviation]
    bufferOffset = _serializer.float64(obj.noise_standard_deviation, buffer, bufferOffset);
    // Serialize message field [amplitude]
    bufferOffset = _serializer.float64(obj.amplitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TutorialSineWaveInput
    let len;
    let data = new TutorialSineWaveInput(null);
    // Deserialize message field [period]
    data.period = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [noise_standard_deviation]
    data.noise_standard_deviation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [amplitude]
    data.amplitude = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_tutorial/TutorialSineWaveInput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cd899687a4aae1254b2449fcada7c3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 period
    float64 noise_standard_deviation
    float64 amplitude
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TutorialSineWaveInput(null);
    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = 0.0
    }

    if (msg.noise_standard_deviation !== undefined) {
      resolved.noise_standard_deviation = msg.noise_standard_deviation;
    }
    else {
      resolved.noise_standard_deviation = 0.0
    }

    if (msg.amplitude !== undefined) {
      resolved.amplitude = msg.amplitude;
    }
    else {
      resolved.amplitude = 0.0
    }

    return resolved;
    }
};

module.exports = TutorialSineWaveInput;
