// Auto-generated. Do not edit!

// (in-package robot_planning.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class CartesianInterpolationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.branch_id = null;
      this.joint_angles = null;
      this.start_pose = null;
      this.goal_pose = null;
      this.duration = null;
      this.frequency = null;
    }
    else {
      if (initObj.hasOwnProperty('branch_id')) {
        this.branch_id = initObj.branch_id
      }
      else {
        this.branch_id = 0;
      }
      if (initObj.hasOwnProperty('joint_angles')) {
        this.joint_angles = initObj.joint_angles
      }
      else {
        this.joint_angles = [];
      }
      if (initObj.hasOwnProperty('start_pose')) {
        this.start_pose = initObj.start_pose
      }
      else {
        this.start_pose = [];
      }
      if (initObj.hasOwnProperty('goal_pose')) {
        this.goal_pose = initObj.goal_pose
      }
      else {
        this.goal_pose = [];
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianInterpolationRequest
    // Serialize message field [branch_id]
    bufferOffset = _serializer.int32(obj.branch_id, buffer, bufferOffset);
    // Serialize message field [joint_angles]
    bufferOffset = _arraySerializer.float64(obj.joint_angles, buffer, bufferOffset, null);
    // Serialize message field [start_pose]
    bufferOffset = _arraySerializer.float64(obj.start_pose, buffer, bufferOffset, null);
    // Serialize message field [goal_pose]
    bufferOffset = _arraySerializer.float64(obj.goal_pose, buffer, bufferOffset, null);
    // Serialize message field [duration]
    bufferOffset = _serializer.float64(obj.duration, buffer, bufferOffset);
    // Serialize message field [frequency]
    bufferOffset = _serializer.float64(obj.frequency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianInterpolationRequest
    let len;
    let data = new CartesianInterpolationRequest(null);
    // Deserialize message field [branch_id]
    data.branch_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint_angles]
    data.joint_angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [start_pose]
    data.start_pose = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [goal_pose]
    data.goal_pose = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [duration]
    data.duration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_angles.length;
    length += 8 * object.start_pose.length;
    length += 8 * object.goal_pose.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_planning/CartesianInterpolationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18b444c6aee4ab28318394690ca28c9f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 branch_id
    float64[] joint_angles
    float64[] start_pose  # [x, y, z, qx, qy, qz, qw]
    float64[] goal_pose   # [x, y, z, qx, qy, qz, qw]
    float64 duration      # 运动时间(秒)
    float64 frequency     # 控制频率(Hz)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianInterpolationRequest(null);
    if (msg.branch_id !== undefined) {
      resolved.branch_id = msg.branch_id;
    }
    else {
      resolved.branch_id = 0
    }

    if (msg.joint_angles !== undefined) {
      resolved.joint_angles = msg.joint_angles;
    }
    else {
      resolved.joint_angles = []
    }

    if (msg.start_pose !== undefined) {
      resolved.start_pose = msg.start_pose;
    }
    else {
      resolved.start_pose = []
    }

    if (msg.goal_pose !== undefined) {
      resolved.goal_pose = msg.goal_pose;
    }
    else {
      resolved.goal_pose = []
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0.0
    }

    return resolved;
    }
};

class CartesianInterpolationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_trajectory = null;
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_trajectory')) {
        this.joint_trajectory = initObj.joint_trajectory
      }
      else {
        this.joint_trajectory = [];
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianInterpolationResponse
    // Serialize message field [joint_trajectory]
    bufferOffset = _arraySerializer.float64(obj.joint_trajectory, buffer, bufferOffset, null);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianInterpolationResponse
    let len;
    let data = new CartesianInterpolationResponse(null);
    // Deserialize message field [joint_trajectory]
    data.joint_trajectory = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_trajectory.length;
    length += _getByteLength(object.message);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_planning/CartesianInterpolationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ba03139604ba18f20c411a4ea79d0af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] joint_trajectory  # Flattened array of joint configurations
    bool success
    string message 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianInterpolationResponse(null);
    if (msg.joint_trajectory !== undefined) {
      resolved.joint_trajectory = msg.joint_trajectory;
    }
    else {
      resolved.joint_trajectory = []
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: CartesianInterpolationRequest,
  Response: CartesianInterpolationResponse,
  md5sum() { return '704fa7f95d1d0a395781e62e40e60c7e'; },
  datatype() { return 'robot_planning/CartesianInterpolation'; }
};
