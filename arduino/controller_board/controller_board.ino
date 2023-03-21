/*
 * Contains the code necessary to communicate with ROS master.
 */
#include "head.hpp"

// This is to make switching between the production Due and other Arduinos used for development easier
#ifdef SerialUSB
  #define SerialConn SerialUSB
#else
  #define SerialConn Serial
#endif

#define BAUD_RATE 57600

// Used for array indexes! Don't change numbers!
enum Joint {
  CHASSIS = 0,
  HEAD_ROLL,
  HEAD_PITCH,
  HEAD_YAW,
  LEFT_EAR_YAW,
  LEFT_EAR_PITCH,
  RIGHT_EAR_YAW,
  RIGHT_EAR_PITCH,
  LEFT_ARM_SHOULDER,
  LEFT_ARM_ROTATOR,
  LEFT_ARM_ELBOW,
  RIGHT_ARM_SHOULDER,
  RIGHT_ARM_ROTATOR,
  RIGHT_ARM_ELBOW,
  NUMBER_OF_JOINTS /* joint state array length, not a valid index! */
};

/*
 * Joint states, as int. In degrees from 0 - 180
 * Indexed by Joint enum
 */
int prev_target_state[Joint::NUMBER_OF_JOINTS];
int target_state[Joint::NUMBER_OF_JOINTS];

void setup() {
  SerialConn.begin(BAUD_RATE, SERIAL_8N1);
  while (!SerialConn) {
    ; // Wait for serial port to connect.
  }
  
  // Run setup for Head
  setupHead();

  // TODO: Run setup for arms & body

  // Send ready packet to master
  //SerialConn.println("ready");
}

/**
 * Converts joint state array into string formatted as 0:0:0:...
 */
String serialize_joint_states(int *s) {
  String tmp = "";

  for (int i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
    tmp += s[i];
    if (i + 1 != Joint::NUMBER_OF_JOINTS) {
      tmp += ":"; // Don't add ':' to final joint value
    }
  }
  
  return tmp;
}

/**
 * Converts joint state string to joint state array
 * @param str trimmed string formatted as 0:0:0:0:...
 * @param *s int array to save joint states into
 */
void deserialize_joint_states(String str, int s[]) {
  int delim_idx = -1; // Index of the first delimeter is the start of the string
  
  for (int i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
    
    int next_delim_idx = str.indexOf(':', delim_idx + 1);

    s[i] = (int) str.substring(delim_idx + 1, next_delim_idx).toInt();
    
    delim_idx = next_delim_idx;
  }
}

// Copies target_state into prev_target_state
void save_prev_target_state() {
  for (int i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
    prev_target_state[i] = target_state[i];
  }
}

// Handles commands sent from serial port.
void cmd_handler() {
  if (SerialConn.available() > 0) {
    String cmd = SerialConn.readStringUntil('\r');
    cmd.trim();
    if (cmd.startsWith("s ")) {
      String raw_values = cmd.substring(2);
      raw_values.trim();
      save_prev_target_state();
      deserialize_joint_states(raw_values, target_state);
      handle_new_target();
      // Acknowledge command
      //SerialConn.print("ack: ");
      //SerialConn.println(cmd);
    } else if (cmd.startsWith("r")) {
      // WIP: If ros doesn't smooth positions well and we need to use velocity as well,
      //      then we need to read current_state, instead of target.
      String state_string = serialize_joint_states(target_state);
      //SerialConn.println(state_string);
    } else {
      //SerialConn.print("[Error]: Invalid Command: ");
      //SerialConn.println(cmd);
    }
  }
}

void handle_new_target() {

  // If one of the head joints has been updated, execute.
  if (head_has_update()) {
    // iterations are 0 bkz we're assuming that ROS is sending real-time positions that will do its own smoothing.
    runHead(
      0, /* Not using head z */
      (double) target_state[Joint::HEAD_ROLL],
      (double) target_state[Joint::HEAD_PITCH],
      (double) target_state[Joint::HEAD_YAW],
      target_state[Joint::LEFT_EAR_YAW],
      target_state[Joint::LEFT_EAR_PITCH],
      target_state[Joint::RIGHT_EAR_YAW],
      target_state[Joint::RIGHT_EAR_PITCH],
      0 /* 0 iterations */
    );
  }
}

#define NUM_OF_HEAD_JOINTS 7
int head_joint_idxs[NUM_OF_HEAD_JOINTS] = {
  Joint::HEAD_ROLL,
  Joint::HEAD_PITCH,
  Joint::HEAD_YAW,
  Joint::LEFT_EAR_YAW,
  Joint::LEFT_EAR_PITCH,
  Joint::RIGHT_EAR_YAW,
  Joint::RIGHT_EAR_PITCH
};

bool head_has_update() {
  for (int i = 0; i < NUM_OF_HEAD_JOINTS; i++) {
    int joint_idx = head_joint_idxs[i];
    if (prev_target_state[joint_idx] != target_state[joint_idx]) {
      return true;
    }
  }
  return false;
}

void loop() {
  cmd_handler();   
}
