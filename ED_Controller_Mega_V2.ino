// Complete code for control box including joystick inputs, button inputs, and XBee communication
// Uses Arduino MEGA with XBee Pro S3B
//
// National Geographic Photo Engineering
//
// Matt Norman
// 18 July, 2022

#include <Wire.h>
#include <PacketSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "switchReading.h"

// Communication period frequency
#define LOOP_PERIOD_MS 50  //

// pin assignments
#define X_DIR_JOYSTICK_PIN A0
#define Y_DIR_JOYSTICK_PIN A1
#define PAN_JOYSTICK_PIN A2
#define TILT_JOYSTICK_PIN A3
#define ROLL_JOYSTICK_PIN A4

// bit mapping
#define X_DIR_JOYSTICK_BIT 0
#define Y_DIR_JOYSTICK_BIT 1
#define PAN_JOYSTICK_BIT 2
#define TILT_JOYSTICK_BIT 3
#define ROLL_JOYSTICK_BIT 4

// Joystick parameters
#define NOISE_TOLERANCE 5

// define packet types
#define JOYSTICK_TYPE 1
#define BUTTON_CMD 2
#define SEND_CMD 3

//OLED screen parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

//global variables
uint8_t sendMsg[4] = { 0x00 };
uint8_t recMsg[4] = { 0x00 };
unsigned long timerVar = 0;
uint16_t switchCheck = 0;

// struct declaration
struct PacketStruct {
  struct Header {
    uint8_t type;
    uint8_t size;
  } header;
  uint8_t data[40];
};

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
JoystickData joystickPkt;
PacketSerial packetSerialStream;
switchReading buttons;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Function prototypes
void sendButtonCmd(uint16_t);
void sendJoystickValue(void);
void onPacketReceived(const uint8_t*, size_t);
void processReceivedData(const uint8_t*, size_t);
void setupOLED(uint8_t);
uint8_t sampleJoystickPins();
uint8_t sampleJoystickPin(uint8_t, uint16_t*);

//Setup loop, runs once
void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  // setup OLED screen and confirm status
  setupOLED(0x3C);

  // setup switches according to parameters set in switches struct
  buttons.switchSetup();

  // initialize joystick data state variables
  sampleJoystickPins();

  // If we want to receive packets, we must specify a packet handler function.
  // The packet handler is a custom function with a signature like the
  // onPacketReceived function below.
  packetSerialStream.setStream(&Serial1);
  packetSerialStream.setPacketHandler(&onPacketReceived);
}

//Main loop, keeps running
void loop() {

  timerVar = millis();

  // // /* sample joystick pins and return state variable that indicates
  // //    if anything has changed above NOISE_TOLERANCE threshold */
  joystickPkt.data.state = sampleJoystickPins();

  // // // /* if change has been detected, send the new measured values to receiver */
  if (joystickPkt.data.state) {
    sendJoystickValue();
  }

  /* poll buttons to check for any changes */
  switchCheck = buttons.pollSwitches();

  if (switchCheck != 0) {
    Serial.println(switchCheck, HEX);
    sendButtonCmd(switchCheck);
  }

  // /* call the packetSerialStream periodically to receive and process incoming packets */
  packetSerialStream.update();

  /* delay until next period */
  if (millis() - timerVar < LOOP_PERIOD_MS) {
    delay(LOOP_PERIOD_MS - (millis() - timerVar));
  }
}

void sendButtonCmd(uint16_t cmd) {
  sentPkt.header.type = BUTTON_CMD;
  sentPkt.header.size = sizeof(cmd);
  memcpy(&sentPkt.data, (uint8_t*)&cmd, sizeof(cmd));
  packetSerialStream.send((uint8_t*)&sentPkt, sentPkt.header.size + sizeof(sentPkt.header));
}


void sendJoystickValue(void) {
  sentPkt.header.type = JOYSTICK_TYPE;
  sentPkt.header.size = sizeof(joystickPkt.data);
  memcpy(&sentPkt.data, &joystickPkt.data_8, sizeof(joystickPkt));
  packetSerialStream.send((uint8_t*)&sentPkt, sentPkt.header.size + sizeof(sentPkt.header));
}


// This is our handler callback function.
// When an encoded packet is received and decoded, it will be delivered here.
// The `buffer` is a pointer to the decoded byte array. `size` is the number of
// bytes in the `buffer`.
void onPacketReceived(const uint8_t* buffer, size_t size) {
  memcpy(&receivedPkt.header, buffer, sizeof(receivedPkt.header));

  /* received data from vehicle */
  if (receivedPkt.header.type == SEND_CMD) {
    processReceivedData(buffer + sizeof(receivedPkt.header), size - sizeof(receivedPkt.header));
  }
}

void processReceivedData(const uint8_t* buffer, size_t size) {
  /* received data from vehicle */
  memcpy(&recMsg, buffer, size);
  recMsg[size] = 0;  //ensure null terminator

  for (uint8_t i = 0; i < sizeof(recMsg); i++) {
    Serial.print(buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// sample all analog joystick pins and update relevant bits in pinState to indicate change
uint8_t sampleJoystickPins() {
  uint8_t pinState = 0;

  pinState |= sampleJoystickPin(X_DIR_JOYSTICK_PIN, &joystickPkt.data.x_dir) << X_DIR_JOYSTICK_BIT;
  pinState |= sampleJoystickPin(Y_DIR_JOYSTICK_PIN, &joystickPkt.data.y_dir) << Y_DIR_JOYSTICK_BIT;
  pinState |= sampleJoystickPin(PAN_JOYSTICK_PIN, &joystickPkt.data.pan) << PAN_JOYSTICK_BIT;
  pinState |= sampleJoystickPin(TILT_JOYSTICK_PIN, &joystickPkt.data.tilt) << TILT_JOYSTICK_BIT;
  pinState |= sampleJoystickPin(ROLL_JOYSTICK_PIN, &joystickPkt.data.roll) << ROLL_JOYSTICK_BIT;

  return pinState;
}

// the below function samples the relevant joystick pin,
// updates the stored value, and
// returns a true if the analog reading is above a threshold
uint8_t sampleJoystickPin(uint8_t pin, uint16_t* oldSample) {

  // grab new sample
  uint16_t newSample = analogRead(pin);

  // check if new sample is above noise threshold
  if (abs(newSample - *oldSample) > NOISE_TOLERANCE) {
    // update oldSample
    *oldSample = newSample;

    return 1;
  }

  return 0;
}

void setupOLED(uint8_t ID) {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, ID)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }

  delay(2000);          // wait for initializing
  oled.clearDisplay();  // clear display

  oled.setTextSize(1);           // text size
  oled.setTextColor(WHITE);      // text color
  oled.setCursor(0, 10);         // position to display
  oled.println("Hello World!");  // text to display
  oled.display();                // show on OLED
}