// Complete code for reciever/vehicle including motor driver control systems, XBee communication, and camera/media systems control
// Uses Teensy 4.1 with Pololu M3S256 for turret control and Sabertooth 2x25 for tank base, optical encoders, XBee Pro S3B
//
// National Geographic Photo Engineering
//
// Matt Norman
// 18 July, 2022

#include <PacketSerial.h>
#include <Motoron.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <DifferentialSteering.h>
#include <Encoder.h>
#include "AirCommanderControl.h"
#include "controllerLayout.h"

#define LOOP_PERIOD_MS 10000
unsigned long timeCount = 0;

// define packet types
#define JOYSTICK_TYPE 1
#define BUTTON_CMD 2
#define SEND_CMD 3

// bit mapping
#define X_DIR_JOYSTICK_BIT 0
#define Y_DIR_JOYSTICK_BIT 1
#define PAN_JOYSTICK_BIT 2
#define TILT_JOYSTICK_BIT 3
#define ROLL_JOYSTICK_BIT 4

//camera defines
#define CAMERA_ALPHA 0x00
#define CAMERA_BETA 0x01
bool shutterType = false;

//Motron I2C initalization
//I2C serial address set to 17 from defult 16, done using setI2C example in Motron library
MotoronI2C mc1(17);  // Use I2C 2 as the pins 24 & 25

//Sabertooth serial initaliztaoin
SabertoothSimplified ST(Serial1);  // Use Serial1 as the serial port pin 1

//Air Commander initalization
AirCommanderControl camCtrlAlpha(&Serial7);  // Use Serial7 as the serial port pins 28 & 29
AirCommanderControl camCtrlBeta(&Serial8);   // Use Serial8 as the serial port pins 34 & 35

//Differential Steering object
DifferentialSteering DiffSteer;

//Create encoder objest using 2 pins that are interupt capable
Encoder panEnc(18, 19);
Encoder tiltEnc(31, 32);

//define relay pins
const uint8_t VIDEO_TRANSMIT_POWER_PIN = 14;
const uint8_t DRIVE_POWER_PIN = 15;
const uint8_t HDMI_SWITCH_TRIGGER = 30;
const uint8_t RIGHT_CAMERA_POWER_PIN = 36;
const uint8_t LEFT_CAMERA_POWER_PIN = 37;
const uint8_t RIGHT_LIGHT_POWER_PIN = 38;
const uint8_t LEFT_LIGHT_POWER_PIN = 39;
const uint8_t MICROPHONE_RECORD_PIN = 40;
const uint8_t EXTRA_9V_OUT_PORT = 41;

//Packet structure defination
struct PacketStruct {
  struct Header {
    uint8_t type;
    uint8_t size;
  } header;
  uint8_t* data;
};

//Joystick data structure defination
union JoystickData {
  struct Data {
    uint16_t state;
    uint16_t x_dir;
    uint16_t y_dir;
    uint16_t pan;
    uint16_t tilt;
    uint16_t roll;
  } data;

  uint8_t data_8;
};

//instances from libraries
PacketStruct sentPkt;
PacketStruct receivedPkt;
PacketSerial packetSerialStream;
JoystickData joystickPkt;

//Camera command message
uint8_t buttonCmd[4] = { 0x00 };
uint8_t sendMsg[4] = { 0x00 };

// Define which status flags the Motoron should treat as errors.
const uint16_t errorMask =
  (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR) | (1 << MOTORON_STATUS_FLAG_CRC_ERROR) | (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED) | (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED) | (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED) | (1 << MOTORON_STATUS_FLAG_RESET) | (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT) | (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING) | (1 << MOTORON_STATUS_FLAG_NO_POWER) | (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE) | (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED) | (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING);

//Function prototypes
bool axisModified(JoystickData*, uint8_t);
int mapVal(double, double, double, double, double);
void setupMotoron(MotoronI2C&);
void setupRelay();
void triggerRelay(uint8_t);
void serialEvent7();
void serialEvent8();
void sendCmd(uint16_t);
void processButtonCmd(const uint8_t*, size_t);
void camCtrl(AirCommanderControl*, uint8_t);
void processJoystickData(const uint8_t*, size_t);
void checkMcError(uint16_t, MotoronI2C&);
void resetMotoron(MotoronI2C&);
void checkAxisStop(int, int, int, int, int, MotoronI2C&);

//Setup loop, runs once
void setup() {
  Wire.begin();           //I2C to Motron
  Serial.begin(9600);     //Serial to pc termianl
  Serial1.begin(9600);    // Serial to Sabertooth
  Serial5.begin(115200);  // Serial to XBee
  camCtrlAlpha.begin();   // Serial to Air Commander for camera one/alpha
  camCtrlBeta.begin();    // Serial to Air Commander for camera two/beta

  DiffSteer.begin(32);

  setupMotoron(mc1);
  delay(50);

  setupRelays();
  // ST.motor(1, 0);
  // delay(10);
  // ST.motor(2, 0);
  // delay(10);

  // If we want to receive packets, we must specify a packet handler function.
  // The packet handler is a custom function with a signature like the
  // onPacketReceived function below.

  packetSerialStream.setStream(&Serial8);
  packetSerialStream.setPacketHandler(&onPacketReceived);

  Serial.print("setup done");
}

//Main loop, keeps running
void loop() {

  /* call the packetSerialStream periodically to receive and process incoming packets */


  //  if (millis() % LOOP_PERIOD_MS == 0) {
  //    sendCmd(1000);
  //    delay(5);
  //  }

  packetSerialStream.update();
}

//---------------------------------------------------------
//function declaration
//---------------------------------------------------------

void sendCmd(uint16_t cmd) {
  sentPkt.header.type = SEND_CMD;
  sentPkt.header.size = sizeof(cmd);
  memcpy(&sentPkt.data, (uint8_t*)&cmd, sizeof(cmd));
  packetSerialStream.send((uint8_t*)&sentPkt, sentPkt.header.size + sizeof(sentPkt.header));
}

// This is our handler callback function.
// When an encoded packet is received and decoded, it will be delivered here.
// The `buffer` is a pointer to the decoded byte array. `size` is the number of
// bytes in the `buffer`.
void onPacketReceived(const uint8_t* buffer, size_t size) {
  memcpy(&receivedPkt.header, buffer, sizeof(receivedPkt.header));
  /* received joystick data */
  if (receivedPkt.header.type == JOYSTICK_TYPE) {
    processJoystickData(buffer + sizeof(receivedPkt.header), size - sizeof(receivedPkt.header));
  }

  if (receivedPkt.header.type == BUTTON_CMD) {

    processButtonCmd(buffer + sizeof(receivedPkt.header), size - sizeof(receivedPkt.header));
  }
}

void processButtonCmd(const uint8_t* buffer, size_t size) {
  /* received joystick data */
  memcpy(&buttonCmd, buffer, size);
  buttonCmd[size] = 0;  //ensure null terminator

  if (buttonCmd[1] == CAMERA_ALPHA) {
    camCtrl(&camCtrlAlpha, buttonCmd[0]);
  } else {  //CAMERA_BETA
    camCtrl(&camCtrlBeta, buttonCmd[0]);
  }
}

void camCtrl(AirCommanderControl* camera, uint8_t command) {
  Serial.print("recieved command:\t");
  Serial.println(command, HEX);

  switch (command) { //Different cases come from controllerLayout.h in switchreadinglibrary 
    case APERTURE_INC:
      camera->aperture_p();
      break;

    case APERTURE_DEC:
      camera->aperture_n();
      break;

    case ISO_INC:
      camera->c1();
      break;

    case ISO_DEC:
      camera->c2();
      break;

    case SHUTTER_SPEED_INC:
      camera->speed_p();
      break;

    case SHUTTER_SPEED_DEC:
      camera->speed_n();
      break;

    case MENU:
      camera->menu();
      break;

    case UP_ARROW:
      camera->press_up();
      break;

    case DOWN_ARROW:
      camera->press_down();
      break;

    case LEFT_ARROW:
      camera->press_left();
      break;

    case RIGHT_ARROW:
      camera->press_right();
      break;

    case SELECT:
      camera->press_ok();
      break;

    case PLAY_BACK:
      camera->display();
      break;

    case TRIGGER_AF_RIGHT:
      camCtrlAlpha.af();
      break;

    case TRIGGER_AF_LEFT:
      camCtrlBeta.af();
      break;

    case SHUTTER_RIGHT:
      if (!shutterType) {
        camCtrlAlpha.photo();
      } else if (shutterType) {
        camCtrlAlpha.burst();
      } else {
        camCtrlAlpha.photo();
      }
      break;

    case SHUTTER_LEFT:
      if (!shutterType) {
        camCtrlBeta.photo();
      } else if (shutterType) {
        camCtrlBeta.burst();
      } else {
        camCtrlBeta.photo();
      }
      break;

    case RECORD_VIDEO:
      camera->video();
      break;

    case CUSTOM_ONE:
      camera->zoom_p();
      break;

    case CUSTOM_TWO:
      camera->zoom_n();
      break;

    case CUSTOM_THREE:
      camera->c3();
      break;

    case CUSTOM_FOUR:
      camera->af_mf();
      break;

    case CAMERA_POWER_RIGHT:
      triggerRelay(RIGHT_CAMERA_POWER_PIN);
      break;

    case CAMERA_POWER_LEFT:
      triggerRelay(LEFT_CAMERA_POWER_PIN);
      break;

    case CAMERA_SELECT:
      triggerRelay(HDMI_SWITCH_TRIGGER);
      break;

    case LIGHT_POWER_RIGHT:
      triggerRelay(RIGHT_LIGHT_POWER_PIN);
      break;

    case LIGHT_POWER_LEFT:
      triggerRelay(LEFT_LIGHT_POWER_PIN);
      break;

    case MICROPHONE_RECORD:
      triggerRelay(MICROPHONE_RECORD_PIN);
      break;

    case VIDEO_TRANSMIT_POWER:
      triggerRelay(VIDEO_TRANSMIT_POWER_PIN);
      break;

    case DRIVE_POWER:
      triggerRelay(DRIVE_POWER_PIN);
      break;

    case MISC_SWITCH:
      triggerRelay(EXTRA_9V_OUT_PORT);
      break;

    case SHUTTER_SELECT:
      shutterType = !shutterType;
      break;
    default:
      Serial.print("error flag");
  }
}

void processJoystickData(const uint8_t* buffer, size_t size) {

  /* received joystick data */
  memcpy(&joystickPkt, buffer, size);
  //  Serial.println(sizeof(joystickPkt));
  //
  //  for(int i = 0; i<sizeof(joystickPkt); i++){
  //    Serial.print(buffer[i], HEX);
  //  }
  //  Serial.println();

  int rightMotor, leftMotor, panMotor, tiltMotor, rollMotor;

  /* check which axis was modified and operate accordingly */
  if (axisModified(&joystickPkt, X_DIR_JOYSTICK_BIT) || axisModified(&joystickPkt, Y_DIR_JOYSTICK_BIT)) {

    Serial.println("wheel movement");

    rightMotor = mapVal(joystickPkt.data.x_dir, 375, 650, -127, 127);
    leftMotor = mapVal(joystickPkt.data.y_dir, 375, 650, -127, 127);

    DiffSteer.computeMotors(rightMotor, leftMotor);

    rightMotor = DiffSteer.computedRightMotor();
    leftMotor = DiffSteer.computedLeftMotor();

    ST.motor(1, rightMotor);
    delay(5);
    ST.motor(2, leftMotor);

    Serial.print("Right motor speed out: ");
    Serial.println(rightMotor);

    Serial.print("Left motor Speed: ");
    Serial.println(leftMotor);
  }

  if (axisModified(&joystickPkt, PAN_JOYSTICK_BIT)) {

    panMotor = mapVal(joystickPkt.data.pan, 375, 650, -800, 800);

    //    Serial.println(errorStat);

    //    if (errorStat) {
    //      Serial.println("flag pan");
    //      checkMcError(mc1.getLastError(), mc1);
    //    }

    //    checkAxisStop(-5000, 5000, panEnc.read(), 1, panMotor, mc1);
    //    checkMcError(mc1.getStatusFlags(), mc1);

    Serial.print("Pan Speed: ");
    Serial.println(panMotor);

    mc1.setSpeed(1, panMotor);
    delay(1);
  }

  if (axisModified(&joystickPkt, TILT_JOYSTICK_BIT)) {

    tiltMotor = mapVal(joystickPkt.data.tilt, 375, 650, -800, 800);

    //    if (errorStat) {
    //      Serial.println("flag tilt");
    //      checkMcError(errorStat, mc1);
    //    }

    //    checkAxisStop(-10000, 10000, tiltEnc.read(), 2, tiltMotor, mc1);
    //        checkMcError(mc1.getStatusFlags(), mc1);
    //     checkForProblems();

    Serial.print("Tilt Speed: ");
    Serial.println(tiltMotor);

    mc1.setSpeed(2, tiltMotor);
    delay(1);
    //    checkMcError(mc1.getStatusFlags(), mc1);
  }

  if (axisModified(&joystickPkt, ROLL_JOYSTICK_BIT)) {

    rollMotor = mapVal(joystickPkt.data.roll, 375, 650, -800, 800);

    //    if (errorStat) {
    //      Serial.println("flag roll");
    //      checkMcError(errorStat, mc1);
    //    }

    //    checkMcError(mc1.getStatusFlags(), mc1);
    //  checkForProblems();

    Serial.print("Roll motor Speed: ");
    Serial.println(rollMotor);

    mc1.setSpeed(3, rollMotor);
    delay(1);
  }
}

// ADC reference voltage: change to 3300 if using a 3.3 V Arduino.
const uint16_t referenceMv = 3300;
void checkForProblems() {
  uint16_t status = mc1.getStatusFlags();
  checkCommunicationError(mc1.getLastError());
  checkControllerError(status);

  uint32_t voltageMv = mc1.getVinVoltageMv(referenceMv);
  checkCommunicationError(mc1.getLastError());
  checkVinVoltage(voltageMv);
}

void checkCommunicationError(uint8_t errorCode) {
  if (errorCode) {
    while (1) {
      mc1.reset();
      Serial.print(F("Communication error: "));
      Serial.println(errorCode);
      delay(1000);
    }
  }
}

// Minimum allowed VIN voltage.  You can raise this to be closer
// to your power supply's expected voltage.
const uint16_t minVinVoltageMv = 24000;
void checkVinVoltage(uint16_t voltageMv) {
  if (voltageMv < minVinVoltageMv) {
    while (1) {
      mc1.reset();
      Serial.print(F("VIN voltage too low: "));
      Serial.println(voltageMv);
      delay(1000);
    }
  }
}

void checkControllerError(uint16_t status) {
  if (status & errorMask) {
    while (1) {
      // One of the error flags is set.  The Motoron should
      // already be stopping the motors.  We report the issue to
      // the user and send reset commands to be extra careful.
      mc1.reset();
      Serial.print(F("Controller error: 0x"));
      Serial.println(status, HEX);
      delay(1000);
    }
  }
}

bool axisModified(JoystickData* joystickData, uint8_t axis) {
  uint8_t mask = 0x01 << axis;
  return (joystickData->data.state & mask) >> axis;
}

void serialEvent7() {
  camCtrlAlpha.serialHandler();
}

void serialEvent8() {
  camCtrlBeta.serialHandler();
}

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();

  mc.clearResetFlag();

  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_BRAKE);

  mc.setErrorMask(errorMask);

  //  mc.disableCrc();

  mc.disableCommandTimeout();

  mc.clearMotorFaultUnconditional();

  mc.setMaxAcceleration(1, 50);
  mc.setMaxDeceleration(1, 100);

  mc.setMaxAcceleration(2, 150);
  mc.setMaxDeceleration(2, 150);

  mc.setMaxAcceleration(3, 100);
  mc.setMaxDeceleration(3, 100);

  Serial.println("Setup Motron Done");

  // Depending on what was happening before this sketch started,
  // the motors will either be stopped or decelerating.
  // This loop waits for them to stop so that when the rest of
  // the code starts running, it will run from
  // a more predictable starting point.  This is optional.
  while (mc.getMotorDrivingFlag()) {
    mc.clearMotorFault();
  }
}

void checkMcError(uint16_t status, MotoronI2C& mc) {
  Serial.println(mc.getVinVoltageMv(3300));

  if (status & errorMask) {
    while (1) {
      Serial.print("Controller error: 0x");
      Serial.println(status, HEX);
      // One of the error flags is set.  The Motoron should
      // already be stopping the motors.  We report the issue to
      // the user and send reset commands to be extra careful.

      mc.clearMotorFault();

      resetMotoron(mc1);

      //      mc.reset();
      //
      //      mc.clearResetFlag();
      //
      //      mc.disableCrc();
      //
      //      mc.clearMotorFaultUnconditional();
      //
      //      delay(1000);
      //
      //      Serial.println("reset done");
    }
  }
}

void resetMotoron(MotoronI2C& mc) {
  mc.reset();

  mc.clearResetFlag();

  mc.clearMotorFaultUnconditional();

  delay(1000);

  Serial.println("reset done");
}

void checkAxisStop(int ccwStop, int cStop, int currentPos, int motorNum, int motorSpeed, MotoronI2C& mc) {

  if (cStop > currentPos && ccwStop < currentPos) {
    mc.setSpeed(motorNum, motorSpeed);
  } else if (currentPos <= ccwStop) {
    //checkCcwStop = true;
    mc.setSpeed(motorNum, -motorSpeed / 4);
    delay(750);
  } else if (currentPos >= cStop) {
    //checkCStop = true;
    mc.setSpeed(motorNum, -motorSpeed / 4);
    delay(750);
  } else {
    mc.setSpeed(motorNum, 0);
  }
}

int mapVal(double valIn, double oldMin, double oldMax, double newMin, double newMax) {
  double oldRange = oldMax - oldMin;
  double newRange = newMax - newMin;
  double valOut = 0.0;

  valOut = (((valIn - oldMin) * newRange) / oldRange) + newMin;

  if (valIn >= 500.0 && valIn <= 550.0) {
    valOut = 0.0;
  }

  if (valOut > newMax) {
    valOut = newMax;
  }

  if (valOut < newMin) {
    valOut = newMin;
  }

  return valOut;
}

void setupRelays() {
  pinMode(VIDEO_TRANSMIT_POWER_PIN, OUTPUT);
  digitalWrite(VIDEO_TRANSMIT_POWER_PIN, HIGH);

  pinMode(DRIVE_POWER_PIN, OUTPUT);
  digitalWrite(DRIVE_POWER_PIN, HIGH);

  pinMode(HDMI_SWITCH_TRIGGER, OUTPUT);
  digitalWrite(HDMI_SWITCH_TRIGGER, HIGH);

  pinMode(RIGHT_CAMERA_POWER_PIN, OUTPUT);
  digitalWrite(RIGHT_CAMERA_POWER_PIN, HIGH);

  pinMode(LEFT_CAMERA_POWER_PIN, OUTPUT);
  digitalWrite(LEFT_CAMERA_POWER_PIN, HIGH);

  pinMode(RIGHT_LIGHT_POWER_PIN, OUTPUT);
  digitalWrite(RIGHT_LIGHT_POWER_PIN, HIGH);

  pinMode(LEFT_LIGHT_POWER_PIN, OUTPUT);
  digitalWrite(LEFT_LIGHT_POWER_PIN, HIGH);

  pinMode(MICROPHONE_RECORD_PIN, OUTPUT);
  digitalWrite(MICROPHONE_RECORD_PIN, HIGH);

  pinMode(EXTRA_9V_OUT_PORT, OUTPUT);
  digitalWrite(EXTRA_9V_OUT_PORT, HIGH);
}

void triggerRelay(uint8_t pin) {
  bool status = digitalRead(pin);
  status = !status;
  digitalWrite(pin, status);
}