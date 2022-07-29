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
MotoronI2C mc1(17);  //I2C serial address set to 17 from defult 16, done using setI2C example in Motron library

//Sabertooth serial initaliztaoin
SabertoothSimplified ST(Serial5);  // Use Serial5 as the serial port.

//Air Commander initalization
AirCommanderControl camCtrlAlpha(&Serial6);
AirCommanderControl camCtrlBeta(&Serial7);

//Differential Steering object
DifferentialSteering DiffSteer;

//Create encoder objest using 2 pins that are interupt capable
Encoder panEnc(30, 31);
Encoder tiltEnc(32, 33);

//define relay pins
const uint8_t RIGHT_CAMERA_POWER_PIN = 1;
const uint8_t LEFT_CAMERA_POWER_PIN = 2;
const uint8_t RIGHT_LIGHT_POWER_PIN = 3;
const uint8_t LEFT_LIGHT_POWER_PIN = 4;
const uint8_t CAMERA_SELECT_PIN = 5;
const uint8_t MICROPHONE_RECORD_PIN = 6;
const uint8_t VIDEO_TRANSMIT_POWER_PIN = 7;
const uint8_t DRIVE_POWER_PIN = 8;
const uint8_t RELAY_COUNT = 8;




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
  (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR) |
  (1 << MOTORON_STATUS_FLAG_CRC_ERROR) |
  (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_RESET) |
  (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING) |
  (1 << MOTORON_STATUS_FLAG_NO_POWER) |
  (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING);

//Function prototypes
bool axisModified(JoystickData*, uint8_t);
int mapVal(double, double, double, double, double);
void setupMotoron(MotoronI2C&);
void setupRelay(uint8_t, uint8_t);
void triggerRelay(uint8_t);
void serialEvent6();
void serialEvent7();
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
  Serial5.begin(9600);    // Serial to Sabertooth
  camCtrlAlpha.begin(); // Serial to Air Commander for camera one/alpha
  camCtrlBeta.begin(); // Serial to Air Commander for camera two/beta
  Serial8.begin(115200);  // Serial to XBee

  DiffSteer.begin(32);

  setupMotoron(mc1);

  setupRelays(RIGHT_CAMERA_POWER_PIN, RELAY_COUNT);
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


  if (millis() % LOOP_PERIOD_MS == 0) {
    sendCmd(1000);
    delay(5);
  }

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

  //  for (uint8_t i = 0; i < sizeof(buttonCmd); i++) {
  //    Serial.print(buffer[i], HEX);
  //    Serial.print(" ");
  //  }
  //  Serial.println();

  if (buttonCmd[1] == CAMERA_ALPHA) {
    camCtrl(&camCtrlAlpha, buttonCmd[0]);
  } else { //CAMERA_BETA
    camCtrl(&camCtrlBeta, buttonCmd[0]);
  }

}

void camCtrl(AirCommanderControl* camera, uint8_t command) {
  Serial.print("sending command:\t");
  Serial.println(command, HEX);

  switch (command) {
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

    case TRIGGER_AF:
      camera->af();
      break;

    case SHUTTER:
      if (!shutterType) {
        camera->photo();
      }
      else if (shutterType) {
        camera->burst();
      }
      else {
        camera->photo();
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
      triggerRelay(CAMERA_SELECT_PIN);
      //change LED or soemthing to show which camera you're viewing
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

    rightMotor = mapVal(joystickPkt.data.x_dir, 0, 1024, -127, 127);
    leftMotor = mapVal(joystickPkt.data.y_dir, 0, 1024, -127, 127);

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

    panMotor = mapVal(joystickPkt.data.pan, 0, 1025, -800, 800);

    if (mc1.getLastError()) {
      checkMcError(mc1.getLastError(), mc1);
    }

    checkAxisStop(-10000, 10000, panEnc.read(), 1, panMotor, mc1);
    //    mc1.setSpeed(1, panMotor);
    //    Serial.print("YAW MOVED: ");
    //    Serial.println(panMotor);
  }

  if (axisModified(&joystickPkt, TILT_JOYSTICK_BIT)) {

    tiltMotor = mapVal(joystickPkt.data.tilt, 0, 1025, -800, 800);

    if (mc1.getLastError()) {
      checkMcError(mc1.getLastError(), mc1);
    }

    checkAxisStop(-10000, 10000, tiltEnc.read(), 2, tiltMotor, mc1);
    //    mc1.setSpeed(2, tiltMotor);
    //    Serial.print("PITCH MOVED: ");
    //    Serial.println(tiltMotor);
  }

  if (axisModified(&joystickPkt, ROLL_JOYSTICK_BIT)) {

    rollMotor = mapVal(joystickPkt.data.roll, 0, 1025, -800, 800);

    if (mc1.getLastError()) {
      checkMcError(mc1.getLastError(), mc1);
    }

    mc1.setSpeed(3, rollMotor);

    //    Serial.print("ROLL MOVED: ");
    //    Serial.println(yawMotor);
  }
}

bool axisModified(JoystickData* joystickData, uint8_t axis) {
  uint8_t mask = 0x01 << axis;
  return (joystickData->data.state & mask) >> axis;
}

void serialEvent6() {
  camCtrlAlpha.serialHandler();
}

void serialEvent7() {
  camCtrlBeta.serialHandler();
}

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();

  mc.clearResetFlag();

  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_BRAKE);

  mc.setErrorMask(errorMask);

  mc.disableCrc();

  mc.disableCommandTimeout();

  mc.clearMotorFaultUnconditional();

  mc.setMaxAcceleration(1, 250);
  mc.setMaxDeceleration(1, 250);

  mc.setMaxAcceleration(2, 250);
  mc.setMaxDeceleration(2, 250);

  mc.setMaxAcceleration(3, 250);
  mc.setMaxDeceleration(3, 250);
}

void checkMcError(uint16_t status, MotoronI2C& mc){
  if (status & errorMask)
  {
    while (1)
    {
      // One of the error flags is set.  The Motoron should
      // already be stopping the motors.  We report the issue to
      // the user and send reset commands to be extra careful.
      resetMotoron(mc1);
      //      Serial.print(F("Controller error: 0x"));
      //      Serial.println(status, HEX);
      //      delay(1000);
    }
  }
}

void resetMotoron(MotoronI2C& mc) {
  mc.reset();

  mc.clearResetFlag();

  mc.clearMotorFaultUnconditional();
}

void checkAxisStop(int ccwStop, int cStop, int currentPos, int motorNum, int motorSpeed, MotoronI2C& mc) {

  if (cStop > currentPos && ccwStop < currentPos) {
    mc.setSpeed(motorNum, motorSpeed);
  }
  else if (currentPos <= ccwStop) {
    //checkCcwStop = true;
    mc.setSpeed(motorNum, -motorSpeed / 4);
    delay(750);
  }
  else if (currentPos >= cStop) {
    //checkCStop = true;
    mc.setSpeed(motorNum, -motorSpeed / 4);
    delay(750);
  }
  else {
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

  return valOut;
}

void setupRelays (uint8_t relayNum, uint8_t startPin) {
  for (int pin = startPin; pin <= relayNum; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
}

void triggerRelay (uint8_t pin) {
  bool status = digitalRead (pin);
  status = !status;
  digitalWrite(pin, status);
}
