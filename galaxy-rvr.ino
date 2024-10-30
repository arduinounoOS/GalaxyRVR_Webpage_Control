/*******************************************************************
  The control program of the Ardunio GalaxyRVR.
  
  Please install the SunFounder Controller APP from APP Store(iOS) or Google Play(Android).

  Development test environment:
    - Arduino IDE 2.0.3
  Board tools:
    - Arduino AVR Boards 1.8.3
  Libraries:
    - IRLremote
    - SoftPWM
    - ArduinoJson
    - Sunfounder_AI_Camera

  Version: 1.0.0
    -- https://github.com/sunfounder/galaxy-rvr.git
  
  Documentation:
    -- https://docs.sunfounder.com/projects/galaxy-rvr/en/latest/

  Author: Sunfounder
  Website: https://www.sunfounder.com
           https://docs.sunfounder.com

********************************************************************/


/*
** KYLED chagned power for follow, track, avoid, and voice to 10 from 80
** No connection RGB to red
** No connection delay to 0
*/

#define VERSION "1.1.0"

#include <Arduino.h>
#include <SoftPWM.h>
#include <string.h>

#include "soft_servo.h"
#include "car_control.h"
#include "ir_obstacle.h"
#include "ultrasonic.h"
#include "cmd_code_config.hpp"
#include "SunFounder_AI_Camera.h"
#include "battery.h"

/*************************** Configure *******************************/
/** @name Configure 
 * 
 */
///@{
/** Whether to enable Watchdog */
#define WATCH_DOG 0
#if WATCH_DOG
#include <avr/wdt.h>
#endif

/** Whether to enable TEST mode */
#define TEST 0
#if TEST
#include "test.h"
#endif

/** Whether to enable print Memory Used */
#define MEM 0
#if MEM
// https://github.com/mpflaga/Arduino-MemoryFree
#include <MemoryFree.h>
#include <pgmStrToRAM.h>  // not needed for new way. but good to have for reference.
#endif


/** Configure Wifi mode, SSID, password*/
#define WIFI_MODE WIFI_MODE_AP
#define SSID "GalaxyRVR"
#define PASSWORD "12345678"

// #define WIFI_MODE WIFI_MODE_STA
// #define SSID "iPhone"
// #define PASSWORD "8313453877"

/** Configure product name */
#define NAME "GalaxyRVR"

/** Configure product type */
#define TYPE "GalaxyRVR"

/** Configure websockets port
 * Sunfounder Controller APP fixed using port 8765
*/
#define PORT "8765"


/** Configure the motors speed in different modes */
#define OBSTACLE_AVOID_POWER 10
#define OBSTACLE_FOLLOW_POWER 10
#define VOICE_CONTROL_POWER 10

/** Configure the follow distance of obstacle follow */
#define FOLLOW_DISTANCE 20

/** websocket communication headers */
#define WS_HEADER "WS+"

///@}

/*********************** Global variables ****************************/
/** Instantiate aicamera, a class for serial communication with ESP32-CAM */
AiCamera aiCam = AiCamera(NAME, TYPE);

/* Config Camera Servo */
SoftServo servo;

#define SERVO_PIN 6
#define SERVO_REVERSE false

/* variables of voice control */
char voice_buf_temp[20];
int8_t current_voice_code = -1;
int32_t voice_time = 0;         // uint:s
uint32_t voice_start_time = 0;  // uint:s

/* variables of motors and servo*/
int8_t leftMotorPower = 0;
int8_t rightMotorPower = 0;
uint8_t servoAngle = 24;

/* variable of esp32-cam flash lamp*/
bool cam_lamp_status = false;
int aiCam_lamp_last = 0;
//@}

/*********************** setup() & loop() ************************/
/**
 * setup(), Ardunio main program entrance
 * 
 * Initialization of some peripherals
 */
unsigned long lastReceiveTime = 0;

void setup() {
  int m = millis();
  Serial.begin(115200);
  Serial.print("GalaxyRVR version ");
  Serial.println(VERSION);

  Serial.println(F("Initialzing..."));
#if defined(ARDUINO_AVR_UNO)
  SoftPWMBegin();  // init softpwm, before the motors initialization and the rgb LEDs initialization
#endif
  carBegin();
  irObstacleBegin();
  batteryBegin();
  servo.attach(SERVO_PIN);
  servo.write(24);

#if !TEST
  aiCam.begin(SSID, PASSWORD, WIFI_MODE, PORT);
  aiCam.setOnReceived(onReceive);
#endif

  while (millis() - m < 500) {  // Wait for peripherals to be ready
    delay(1);
  }

#if WATCH_DOG
  wdt_disable();       /* Disable the watchdog and wait for more than 2 seconds */
  delay(3000);         /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
  wdt_enable(WDTO_2S); /* Enable the watchdog with a timeout of 2 seconds */
#endif

  Serial.println(F("Okie!"));
  rgbWrite(GREEN);  // init finished
}

/**
 * loop(), Ardunio main loop
 * 
 * - inclued
 *  - aiCam.loop()
 *  - modeHandler()
 * - or modules test
 */
void loop() {
#if !TEST
  // because the value in a is constantly updated
  // Note that the cycle interval of the "aiCam.loop()" should be less than 80ms to avoid data d
  aiCam.loop();
  if (aiCam.ws_connected == false) {
    currentMode = MODE_DISCONNECT;
    int8_t current_voice_code = -1;
    int8_t voice_time = 0;
    if (currentMode != MODE_DISCONNECT) {

    }
  } else {
    if (currentMode == MODE_DISCONNECT) currentMode = MODE_NONE;
  }

    if (millis() - lastReceiveTime > 180) {
    currentMode = MODE_NONE;  // Stop Mode
    carStop(); // Stop the vehicle
    }

  modeHandler();
#else
  /* Select the item to be tested, multiple selection allowed */
  motors_test();
  // rgb_test();
  // ultrasonic_test();
  // ir_obstacle_test();
  // obstacleAvoidance();
#endif

#if WATCH_DOG
  wdt_reset(); /* Reset the watchdog */
#endif

#if MEM
  Serial.print(F("Free RAM = "));  //F function does the same and is now a built in library, in IDE > 1.0.0
  Serial.println(freeMemory());    // print how much RAM is available in bytes.
#endif
}

/***************************** Functions ******************************/
/**
 * modeHandler(), Execute the corresponding program according to the set mode
 * 
 * - inclued
 *  - MODE_NONE
 *  - MODE_OBSTACLE_FOLLOWING
 *  - MODE_OBSTACLE_AVOIDANCE
 *  - MODE_REMOTE_CONTROL
 *  - MODE_APP_CONTROL
 */

unsigned long previousMillis = 0;  // Variable to store the last time servo movement was updated
const long interval = 10;          // Interval for servo updates in milliseconds (adjust as needed)

int currentServoAngle = 24;        // Current servo angle

void modeHandler() {
  switch (currentMode) {
    case MODE_NONE:
      rgbWrite(MODE_NONE_COLOR);
      carStop();
      servoAngle = 24;  // Set target servo angle for this mode
      break;
    case MODE_DISCONNECT:
      rgbWrite(MODE_DISCONNECT_COLOR);
      //aiCam.lamp_off();
      carStop();
      servoAngle = 24;  // Set target servo angle for this mode
      break;
    case MODE_OBSTACLE_FOLLOWING:
      rgbWrite(MODE_OBSTACLE_FOLLOWING_COLOR);
      obstacleFollowing();
      break;
    case MODE_OBSTACLE_AVOIDANCE:
      rgbWrite(MODE_OBSTACLE_AVOIDANCE_COLOR);
      obstacleAvoidance();
      break;
    case MODE_APP_CONTROL:
      rgbWrite(MODE_APP_CONTROL_COLOR);
      carSetMotors(leftMotorPower, rightMotorPower);
      break;
    case MODE_VOICE_CONTROL:
      rgbWrite(MODE_VOICE_CONTROL_COLOR);
      voice_control();
      break;
    default:
      break;
  }

  // Non-blocking delay implementation for servo movement
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Smoothly update the current servo angle towards the target servo angle
    int velocity = 1;  // Degree increment step, adjust for desired speed
    if (currentServoAngle < servoAngle) {
      currentServoAngle = min(currentServoAngle + velocity, servoAngle);
    } else if (currentServoAngle > servoAngle) {
      currentServoAngle = max(currentServoAngle - velocity, servoAngle);
    }

    // Write the updated angle to the servo
    servo.write(currentServoAngle);
  }
}
// void modeHandler() {
//   switch (currentMode) {
//     case MODE_NONE:
//       rgbWrite(MODE_NONE_COLOR);
//       carStop();
//       servoAngle = 24;
//       servo.write(servoAngle);
//       break;
//     case MODE_DISCONNECT:
//       // if (millis() - rgb_blink_start_time > rgb_blink_interval) {
//       //   rgb_blink_flag = !rgb_blink_flag;
//       //   rgb_blink_start_time = millis();
//       // }
//       // if (rgb_blink_flag) rgbWrite(MODE_DISCONNECT_COLOR);
//       // else rgbOff();
//       rgbWrite(MODE_DISCONNECT_COLOR);
//       carStop();
//       servoAngle = 24;
//       servo.write(servoAngle);
//       break;
//     case MODE_OBSTACLE_FOLLOWING:
//       rgbWrite(MODE_OBSTACLE_FOLLOWING_COLOR);
//       servo.write(servoAngle);
//       obstacleFollowing();
//       break;
//     case MODE_OBSTACLE_AVOIDANCE:
//       rgbWrite(MODE_OBSTACLE_AVOIDANCE_COLOR);
//       servo.write(servoAngle);
//       obstacleAvoidance();
//       break;
//     case MODE_APP_CONTROL:
//       rgbWrite(MODE_APP_CONTROL_COLOR);
//       servo.write(servoAngle);
//       carSetMotors(leftMotorPower, rightMotorPower);
//       break;
//     case MODE_VOICE_CONTROL:
//       rgbWrite(MODE_VOICE_CONTROL_COLOR);
//       servo.write(servoAngle);
//       voice_control();
//       break;
//     default:
//       break;
//   }
// }


/**
 * Obstacle follow program
 */
void obstacleFollowing() {
  byte result = irObstacleRead();
  bool leftIsClear = result & 0b00000010;
  bool rightIsClear = result & 0b00000001;
  float usDistance = ultrasonicRead();
  // usDistance = -1 while the distance is too far
  if (usDistance < 4 && usDistance > 0) {
    carStop();
  } else if (usDistance < 10 && usDistance > 0) {
    carForward(30);
  } else if (usDistance < FOLLOW_DISTANCE && usDistance > 0) {
    carForward(OBSTACLE_FOLLOW_POWER);
  } else {
    if (!leftIsClear) {
      carTurnLeft((int8_t)OBSTACLE_FOLLOW_POWER);
    } else if (!rightIsClear) {
      carTurnRight(OBSTACLE_FOLLOW_POWER);
    } else {
      carStop();
    }
  }
}

/**
 * Obstacle avoidance program
 */
int8_t last_clear = -1;  // last_clear, 1, left; -1, right;
bool last_forward = false;

void obstacleAvoidance() {
  byte result = irObstacleRead();
  bool leftIsClear = result & 0b00000010;   // left, clear: True
  bool rightIsClear = result & 0b00000001;  // right, clear: True
  bool middleIsClear = ultrasonicIsClear();

  if (middleIsClear && leftIsClear && rightIsClear) {  // 111
    last_forward = true;
    carForward(OBSTACLE_AVOID_POWER);
  } else {
    if ((leftIsClear && rightIsClear) || (!leftIsClear && !rightIsClear)) {  // 101, 000, 010
      if (last_clear == 1) carTurnLeft(OBSTACLE_AVOID_POWER);
      else carTurnRight(OBSTACLE_AVOID_POWER);
      last_forward = false;
    } else if (leftIsClear) {  // 100, 110
      if (last_clear == 1 || last_forward == true) {
        carTurnLeft(OBSTACLE_AVOID_POWER);
        last_clear = 1;
        last_forward = false;
      }
    } else if (rightIsClear) {  // 001, 011
      if (last_clear == -1 || last_forward == true) {
        carTurnRight(OBSTACLE_AVOID_POWER);
        last_clear = -1;
        last_forward = false;
      }
    }
  }
}

/**
 * voice control program
 */
void voice_control() {
  if (voice_time == -1) {
    voice_action(current_voice_code, VOICE_CONTROL_POWER);
  } else {
    if (millis() - voice_start_time <= voice_time) {
      voice_action(current_voice_code, VOICE_CONTROL_POWER);
    } else {
      currentMode = MODE_NONE;
      voice_start_time = 0;
      current_voice_code = -1;
    }
  }
}

/**
 * websocket received data processing
 */
void onReceive() {
  lastReceiveTime = millis();
  // --------------------- send data ---------------------
  // battery voltage
  // Serial.print(F("voltage:"));Serial.println(batteryGetVoltage());
  aiCam.sendDoc["BV"] = batteryGetVoltage();

  // IR obstacle detection data
  byte result = irObstacleRead();
  aiCam.sendDoc["N"] = int(!bool(result & 0b00000010));  // left, clear:0
  aiCam.sendDoc["P"] = int(!bool(result & 0b00000001));  // right, clear:0

  // ultrasonic
  float usDistance = int(ultrasonicRead() * 100) / 100.0;  // round two decimal places
  aiCam.sendDoc["O"] = usDistance;

  // --------------------- get data ---------------------
  // Stop
  if (aiCam.getButton(REGION_I)) {
    currentMode = MODE_NONE;
    current_voice_code = -1;
    voice_time = 0;
    carStop();
    return;
  }

  // Mode select: obstacle following, obstacle avoidance
  if (aiCam.getSwitch(REGION_E)) {
    if (currentMode != MODE_OBSTACLE_AVOIDANCE) {
      currentMode = MODE_OBSTACLE_AVOIDANCE;
    }
  } else if (aiCam.getSwitch(REGION_F)) {
    if (currentMode != MODE_OBSTACLE_FOLLOWING) {
      currentMode = MODE_OBSTACLE_FOLLOWING;
    }
  } else {
    if (currentMode == MODE_OBSTACLE_FOLLOWING || currentMode == MODE_OBSTACLE_AVOIDANCE) {
      currentMode = MODE_NONE;
      carStop();
      return;
    }
  }

  // cam lamp
  // if (aiCam.getSwitch(REGION_M) && !cam_lamp_status) {
  //   Serial.println("lamp on");
  //   aiCam.lamp_on(10);  //turn on cam lamp, level 0 ~ 10
  //   cam_lamp_status = true;
  // } else if (!aiCam.getSwitch(REGION_M) && cam_lamp_status) {
  //   Serial.println("lamp off");
  //   aiCam.lamp_off();  // turn off cam lamp
  //   cam_lamp_status = false;
  // }

  int aiCam_lamp_level = aiCam.getSlider(REGION_A);

  if (aiCam_lamp_level != aiCam_lamp_last) {
  if (aiCam_lamp_level > 0) {
    aiCam.lamp_on(aiCam_lamp_level);
  }
  else {aiCam.lamp_off();}
  aiCam_lamp_last = aiCam_lamp_level;
  }

  // Speech control
  if (currentMode != MODE_VOICE_CONTROL) {
    current_voice_code = -1;
    voice_time = 0;
    voice_start_time = 0;
    aiCam.sendDoc["J"] = 0;
  }

  int8_t code = -1;
  voice_buf_temp[0] = 0;  // voice_buf_temp
  aiCam.getSpeech(REGION_J, voice_buf_temp);
  if (strlen(voice_buf_temp) > 0) {
    aiCam.sendDoc["J"] = 1;
    aiCam.sendData();
    aiCam.sendDoc["J"] = 0;
    code = text_2_cmd_code(voice_buf_temp);
    if (code != -1) {
      current_voice_code = code;
      voice_time = voice_action_time[code];
      voice_start_time = millis();
    }
  }

  if (current_voice_code != -1) {
    currentMode = MODE_VOICE_CONTROL;
  }

  // servo angle
  int temp = aiCam.getSlider(REGION_D);
  if (servoAngle != temp) {
    if (currentMode == MODE_NONE || currentMode == MODE_DISCONNECT) {
      currentMode = MODE_APP_CONTROL;
    }
    if (SERVO_REVERSE) {
      temp = constrain(temp, 0, 180);
      temp = 180 - temp;
    } else {
      temp = constrain(temp, 0, 180);
    }
    servoAngle = temp;
  }


  // throttle
  
int throttle_FB = aiCam.getThrottle(REGION_K);  // Forward/Backward throttle input
int throttle_D = aiCam.getThrottle(REGION_Q);   // Directional throttle input
// int16_t throttle_D = aiCam.getJoystick(REGION_Q, JOYSTICK_X);
// int16_t throttle_FB = aiCam.getJoystick(REGION_Q, JOYSTICK_Y);

// Scale directional throttle based on forward/backward throttle if throttle_FB is not zero
// if (throttle_FB != 0) {
// throttle_D = constrain(throttle_D, -abs(throttle_FB), abs(throttle_FB));
// }

int throttle_L, throttle_R;
// Calculate left and right throttle based on adjusted forward/backward and directional inputs
throttle_L = throttle_FB + throttle_D;
throttle_R = throttle_FB - throttle_D;

// Clamp values to acceptable range (assume -255 to 255 for example)
throttle_L = map(constrain(throttle_L, -255, 255),-100,100,-10,10);
throttle_R = map(constrain(throttle_R, -255, 255),-100,100,-10,10);

if (throttle_FB != 0 || throttle_D != 0 || throttle_L != leftMotorPower || throttle_R != rightMotorPower) {
  currentMode = MODE_APP_CONTROL;
  leftMotorPower = throttle_L;
  rightMotorPower = throttle_R;
}
  // int throttle_L = aiCam.getThrottle(REGION_K);
  // int throttle_R = aiCam.getThrottle(REGION_Q);
  // // Serial.print("throttle_L: "); Serial.print(throttle_L);
  // // Serial.print("throttle_R: "); Serial.println(throttle_R);
  // if (throttle_L != 0 || throttle_R != 0 || throttle_L != leftMotorPower || throttle_R != rightMotorPower) {
  //   currentMode = MODE_APP_CONTROL;
  //   leftMotorPower = throttle_L;
  //   rightMotorPower = throttle_R;
  // }
}

