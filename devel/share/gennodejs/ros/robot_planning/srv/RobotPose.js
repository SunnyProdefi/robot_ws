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

class RobotPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.float_base_pose = null;
      this.branch2_joints = null;
      this.branch3_joints = null;
      this.source_frame = null;
      this.target_frame = null;
    }
    else {
      if (initObj.hasOwnProperty('float_base_pose')) {
        this.float_base_pose = initObj.float_base_pose
      }
      else {
        this.float_base_pose = [];
      }
      if (initObj.hasOwnProperty('branch2_joints')) {
        this.branch2_joints = initObj.branch2_joints
      }
      else {
        this.branch2_joints = [];
      }
      if (initObj.hasOwnProperty('branch3_joints')) {
        this.branch3_joints = initObj.branch3_joints
      }
      else {
        this.branch3_joints = [];
      }
      if (initObj.hasOwnProperty('source_frame')) {
        this.source_frame = initObj.source_frame
      }
      else {
        this.source_frame = '';
      }
      if (initObj.hasOwnProperty('target_frame')) {
        this.target_frame = initObj.target_frame
      }
      else {
        this.target_frame = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotPoseRequest
    // Serialize message field [float_base_pose]
    bufferOffset = _arraySerializer.float64(obj.float_base_pose, buffer, bufferOffset, null);
    // Serialize message field [branch2_joints]
    bufferOffset = _arraySerializer.float64(obj.branch2_joints, buffer, bufferOffset, null);
    // Serialize message field [branch3_joints]
    bufferOffset = _arraySerializer.float64(obj.branch3_joints, buffer, bufferOffset, null);
    // Serialize message field [source_frame]
    bufferOffset = _serializer.string(obj.source_frame, buffer, bufferOffset);
    // Serialize message field [target_frame]
    bufferOffset = _serializer.string(obj.target_frame, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotPoseRequest
    let len;
    let data = new RobotPoseRequest(null);
    // Deserialize message field [float_base_pose]
    data.float_base_pose = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [branch2_joints]
    data.branch2_joints = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [branch3_joints]
    data.branch3_joints = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [source_frame]
    data.source_frame = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_frame]
    data.target_frame = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.float_base_pose.length;
    length += 8 * object.branch2_joints.length;
    length += 8 * object.branch3_joints.length;
    length += _getByteLength(object.source_frame);
    length += _getByteLength(object.target_frame);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_planning/RobotPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '07fe2ac8e025e36bfcb59b4b3e359c8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] float_base_pose  # [x, y, z, qx, qy, qz, qw] 相对于world坐标系的位姿
    float64[] branch2_joints   # 分支2的关节角
    float64[] branch3_joints   # 分支3的关节角
    string source_frame        # 源坐标系名称
    string target_frame        # 目标坐标系名称
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotPoseRequest(null);
    if (msg.float_base_pose !== undefined) {
      resolved.float_base_pose = msg.float_base_pose;
    }
    else {
      resolved.float_base_pose = []
    }

    if (msg.branch2_joints !== undefined) {
      resolved.branch2_joints = msg.branch2_joints;
    }
    else {
      resolved.branch2_joints = []
    }

    if (msg.branch3_joints !== undefined) {
      resolved.branch3_joints = msg.branch3_joints;
    }
    else {
      resolved.branch3_joints = []
    }

    if (msg.source_frame !== undefined) {
      resolved.source_frame = msg.source_frame;
    }
    else {
      resolved.source_frame = ''
    }

    if (msg.target_frame !== undefined) {
      resolved.target_frame = msg.target_frame;
    }
    else {
      resolved.target_frame = ''
    }

    return resolved;
    }
};

class RobotPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.transform = null;
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('transform')) {
        this.transform = initObj.transform
      }
      else {
        this.transform = [];
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
    // Serializes a message object of type RobotPoseResponse
    // Serialize message field [transform]
    bufferOffset = _arraySerializer.float64(obj.transform, buffer, bufferOffset, null);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotPoseResponse
    let len;
    let data = new RobotPoseResponse(null);
    // Deserialize message field [transform]
    data.transform = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.transform.length;
    length += _getByteLength(object.message);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_planning/RobotPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '56fdb6d4914112e35dd8259f93da86cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] transform       # [x, y, z, qx, qy, qz, qw] 源坐标系到目标坐标系的变换
    bool success
    string message 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotPoseResponse(null);
    if (msg.transform !== undefined) {
      resolved.transform = msg.transform;
    }
    else {
      resolved.transform = []
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
  Request: RobotPoseRequest,
  Response: RobotPoseResponse,
  md5sum() { return '88bd561bfe99b318bb370cf7b0dd2f58'; },
  datatype() { return 'robot_planning/RobotPose'; }
};
