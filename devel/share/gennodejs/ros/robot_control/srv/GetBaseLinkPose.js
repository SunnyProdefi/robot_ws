// Auto-generated. Do not edit!

// (in-package robot_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetBaseLinkPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angles_branch1 = null;
      this.joint_angles_branch4 = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angles_branch1')) {
        this.joint_angles_branch1 = initObj.joint_angles_branch1
      }
      else {
        this.joint_angles_branch1 = [];
      }
      if (initObj.hasOwnProperty('joint_angles_branch4')) {
        this.joint_angles_branch4 = initObj.joint_angles_branch4
      }
      else {
        this.joint_angles_branch4 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetBaseLinkPoseRequest
    // Serialize message field [joint_angles_branch1]
    bufferOffset = _arraySerializer.float64(obj.joint_angles_branch1, buffer, bufferOffset, null);
    // Serialize message field [joint_angles_branch4]
    bufferOffset = _arraySerializer.float64(obj.joint_angles_branch4, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBaseLinkPoseRequest
    let len;
    let data = new GetBaseLinkPoseRequest(null);
    // Deserialize message field [joint_angles_branch1]
    data.joint_angles_branch1 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_angles_branch4]
    data.joint_angles_branch4 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_angles_branch1.length;
    length += 8 * object.joint_angles_branch4.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_control/GetBaseLinkPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd997c570f966380228b6d56ed45cae97';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] joint_angles_branch1
    float64[] joint_angles_branch4
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetBaseLinkPoseRequest(null);
    if (msg.joint_angles_branch1 !== undefined) {
      resolved.joint_angles_branch1 = msg.joint_angles_branch1;
    }
    else {
      resolved.joint_angles_branch1 = []
    }

    if (msg.joint_angles_branch4 !== undefined) {
      resolved.joint_angles_branch4 = msg.joint_angles_branch4;
    }
    else {
      resolved.joint_angles_branch4 = []
    }

    return resolved;
    }
};

class GetBaseLinkPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base_link_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('base_link_pose')) {
        this.base_link_pose = initObj.base_link_pose
      }
      else {
        this.base_link_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetBaseLinkPoseResponse
    // Serialize message field [base_link_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.base_link_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBaseLinkPoseResponse
    let len;
    let data = new GetBaseLinkPoseResponse(null);
    // Deserialize message field [base_link_pose]
    data.base_link_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_control/GetBaseLinkPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4dd5c128d0b36079967e1c67bbeccd38';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response: 位姿
    geometry_msgs/Pose base_link_pose
    
    
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
    const resolved = new GetBaseLinkPoseResponse(null);
    if (msg.base_link_pose !== undefined) {
      resolved.base_link_pose = geometry_msgs.msg.Pose.Resolve(msg.base_link_pose)
    }
    else {
      resolved.base_link_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetBaseLinkPoseRequest,
  Response: GetBaseLinkPoseResponse,
  md5sum() { return '124f2f1fb4bf270787e14bd14a8d0a4b'; },
  datatype() { return 'robot_control/GetBaseLinkPose'; }
};
