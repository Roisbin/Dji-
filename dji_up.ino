#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <PS4Controller.h>

// ==========================
// MCP2515 PIN & CAN SETUP
// ==========================
#define CAN_CS 5
#define CAN_INT 4

MCP_CAN CAN(CAN_CS);

// ==========================
// MOTOR VARIABLES
// ==========================
int LF = 0, LB = 0, RF = 0, RB = 0; // Motor values
int xData = 0, yData = 0;
int X = 0, Y = 0;
int mode = 0;

// ==========================
// TIMING
// ==========================
unsigned long lastSend = 0;
const unsigned long sendInterval = 5; // Send every 5 ms

// ==========================
// CAN IDs
// ==========================
const int sendID = 0x200;
const int feedbackID_LF = 0x202;
const int feedbackID_LB = 0x204;
const int feedbackID_RF = 0x203;
const int feedbackID_RB = 0x201;

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);

  // CAN setup
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("✅ CAN Initialized Successfully");
  else {
    Serial.println("❌ CAN Initialization Failed");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);

  // PS4 setup
  PS4.begin("18:67:B0:51:0E:1D");
  Serial.println("🎮 PS4 Controller Ready");
}

// ==========================
// MOVEMENT FUNCTIONS
// ==========================
void forward(int speed) {
  LF = -speed;
  //LB = speed;
  RF = -speed;
  RB = speed;
 // Serial.println("Forward");
}

void backward(int speed) {
  LF = speed;
  LB = -speed;
  RF = speed;
  RB = -speed;
 // Serial.println("Backward");
}

void left(int speed) {
  LF = -speed;
  LB = speed;
  RF = -speed;
  RB = speed;
  //Serial.println("Left");
}

void right(int speed) {
  LF = speed;
  LB = -speed;
  RF = -speed;
  RB = speed;
 // Serial.println("Right");
}

void stopAll() {
  LF = LB = RF = RB = 0;
 // Serial.println("Stop");
}

// ==========================
// AXIS LOGIC (R-stick)
// ==========================
void axis_logic(int X, int Y) {
  int deadzone = 20;

  // Deadzone check
  if (abs(X) < deadzone && abs(Y) < deadzone) {
    stopAll();
    return;
  }

  // Forward & backward
  if (Y > deadzone && abs(X) < Y) {
    forward(Y);
  } 
  else if (Y < -deadzone && abs(X) < abs(Y)) {
    backward(-Y);
  }
  // Turning
  else if (X > deadzone && abs(X) > abs(Y)) {
    right(X);
  } 
  else if (X < -deadzone && abs(X) > abs(Y)) {
    left(-X);
  } 
  else {
    stopAll();
  }
}

// ==========================
// CAN SEND FUNCTION
// ==========================
void sendMotorData() {
  byte data[8];

  data[0] = (LB >> 8) & 0xFF;
  data[1] = LB & 0xFF;
  data[2] = (LF >> 8) & 0xFF;
  data[3] = LF & 0xFF;
  data[4] = (RB >> 8) & 0xFF;
  data[5] = RB & 0xFF;
  data[6] = (RF >> 8) & 0xFF;
  data[7] = RF & 0xFF;

  Serial.printf("LF=%d  LB=%d  RF=%d  RB=%d\n", LF, LB, RF, RB);
  
  
  if (CAN.sendMsgBuf(sendID, 0, 8, data) != CAN_OK)
    Serial.println("⚠️ Failed to send command");
}

// ==========================
// LOOP
// ==========================
void loop() {
  // Read joystick
  xData = PS4.RStickX();
  yData = PS4.RStickY();

  // Map joystick to motor command range
  X = map(xData, -128, 127, -1500, 1500);
  Y = map(yData, -128, 127, -1500, 1500);

  // Movement logic
  if (mode == 0) {
    axis_logic(X, Y);
  }

  // Send data to motors
  if (millis() - lastSend >= sendInterval) {
    sendMotorData();
    lastSend = millis();
  }

  // Optional feedback handling
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    byte len;
    byte rxBuf[8];
    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      if (rxId == feedbackID_LF) {
        int16_t rpm = (rxBuf[2] << 8) | rxBuf[3];
        int16_t current = (rxBuf[4] << 8) | rxBuf[5];
        int8_t temp = rxBuf[6];
        // Debug print if needed
        // Serial.printf("RPM: %d | Current: %d | Temp: %d\n", rpm, current, temp);
      }
    }
  }
}
