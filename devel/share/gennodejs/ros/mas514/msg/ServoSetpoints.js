// Auto-generated. Do not edit!

// (in-package mas514.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ServoSetpoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rightWheel = null;
      this.leftWheel = null;
      this.servo1 = null;
      this.servo2 = null;
      this.servo3 = null;
    }
    else {
      if (initObj.hasOwnProperty('rightWheel')) {
        this.rightWheel = initObj.rightWheel
      }
      else {
        this.rightWheel = 0;
      }
      if (initObj.hasOwnProperty('leftWheel')) {
        this.leftWheel = initObj.leftWheel
      }
      else {
        this.leftWheel = 0;
      }
      if (initObj.hasOwnProperty('servo1')) {
        this.servo1 = initObj.servo1
      }
      else {
        this.servo1 = 0;
      }
      if (initObj.hasOwnProperty('servo2')) {
        this.servo2 = initObj.servo2
      }
      else {
        this.servo2 = 0;
      }
      if (initObj.hasOwnProperty('servo3')) {
        this.servo3 = initObj.servo3
      }
      else {
        this.servo3 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ServoSetpoints
    // Serialize message field [rightWheel]
    bufferOffset = _serializer.int64(obj.rightWheel, buffer, bufferOffset);
    // Serialize message field [leftWheel]
    bufferOffset = _serializer.int64(obj.leftWheel, buffer, bufferOffset);
    // Serialize message field [servo1]
    bufferOffset = _serializer.int64(obj.servo1, buffer, bufferOffset);
    // Serialize message field [servo2]
    bufferOffset = _serializer.int64(obj.servo2, buffer, bufferOffset);
    // Serialize message field [servo3]
    bufferOffset = _serializer.int64(obj.servo3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ServoSetpoints
    let len;
    let data = new ServoSetpoints(null);
    // Deserialize message field [rightWheel]
    data.rightWheel = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [leftWheel]
    data.leftWheel = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [servo1]
    data.servo1 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [servo2]
    data.servo2 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [servo3]
    data.servo3 = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mas514/ServoSetpoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '790ff8777d41a4ebf31d8435d32b565b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 rightWheel
    int64 leftWheel
    int64 servo1
    int64 servo2
    int64 servo3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ServoSetpoints(null);
    if (msg.rightWheel !== undefined) {
      resolved.rightWheel = msg.rightWheel;
    }
    else {
      resolved.rightWheel = 0
    }

    if (msg.leftWheel !== undefined) {
      resolved.leftWheel = msg.leftWheel;
    }
    else {
      resolved.leftWheel = 0
    }

    if (msg.servo1 !== undefined) {
      resolved.servo1 = msg.servo1;
    }
    else {
      resolved.servo1 = 0
    }

    if (msg.servo2 !== undefined) {
      resolved.servo2 = msg.servo2;
    }
    else {
      resolved.servo2 = 0
    }

    if (msg.servo3 !== undefined) {
      resolved.servo3 = msg.servo3;
    }
    else {
      resolved.servo3 = 0
    }

    return resolved;
    }
};

module.exports = ServoSetpoints;
