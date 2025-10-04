#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <PS4Controller.h>


// Define MCP2515 pins
#define CAN_CS 5     // Chip Select pin
#define CAN_INT 4    // Interrupt pin


// Motor Speed Value 
int LF = 0;
int LB = 0;
int RF = 0;
int RB = 0;

int xData = 0;
int yData = 0;

int X = 0;
int Y = 0;
int mode = 0;

// Timer
unsigned long last_time = 0;
unsigned long current_time = 0;
float delta_time = 0;

// feedback data
long unsigned int rxId;
byte len = 0;
byte rxBuf[8];

MCP_CAN CAN(CAN_CS);

// Define IDs
const int sendID = 0x200; 
const int feedbackID_LF = 0x202; 
const int feedbackID_LB = 0x204;
const int feedbackID_RF = 0x203; 
const int feedbackID_RB = 0x201; 

unsigned long lastSend = 0;
const unsigned long sendInterval = 1; // 1 ms
double feedback[4];


void setup() {
    Serial.begin(115200);

    // Initialize MCP2515 at 1 Mbps
    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN Initialized Successfully");
    } else {
        Serial.println("CAN Initialization Failed");
        while (1);
    }
    CAN.setMode(MCP_NORMAL);
    PS4.begin("18:67:B0:51:0E:1D");

    // Configure CAN interrupt pin
    pinMode(CAN_INT, INPUT);
    
}

void forward(int X, int Y){
    LF = Y*(-1);
    LB = Y*(-1);
    RF = Y;
    RB = Y;
    Serial.println("Forward");
}

void backward(int X, int Y){
    LF = Y;
    LB = Y;
    RF = Y*(-1);
    RB = Y*(-1);
    Serial.println("Backward");
}

void left(int X, int Y){
    LF = 0;
    LB = 0;
    RF = X;
    RB = X;
    Serial.println("Left");
}

void right(int X, int Y){
    LF = X;
    LB = X;
    RF = 0;
    RB = 0;
    Serial.println("Right");
}

void throttle(int X, int Y){

    LF = Y-X;
    LB = Y-X;
    RF = -Y-X;
    RB = -Y-Y;
}

void axis_logic(int X, int Y){
    // For axis = x(+), y(+) 1
    if (X > 20 & Y > 20){
        if (X > Y){
            right(X, Y);
        }
        else{
            forward((-1)*X, (-1)*Y);
        }
    }

    // For axis = x(+), y(-) 2
    else if (X > 20 & Y < -20){
        if (X > Y*(-1)){
            right(X, Y);
        }
        else{
            backward(X, Y);
        }
    }

    // For axis = x(-), y(+) 3
    else if (X < -20 & Y > 20){
        if (X*(-1) > Y){
            left(X, Y);
        }
        else{
            forward((-1)*X, (-1)*Y);
        }
    }

    // For axis = x(-), y(-) 4
    else if (X < -20 & Y <- 20){
        if (X*(-1) > Y*(-1)){
            left(X, Y);
        }
        else{
            backward(X, Y);
        }
    }
    else {
        Serial.println("Zero");
        LF = 0;
        LB = 0;
        RF = 0;
        RB = 0;
    }
}

void axis_logic2(int X, int Y){
    
}

void loop() {

    xData = PS4.RStickX();
    yData = PS4.RStickY();

    X = map(xData, -130, 130, -1500, 1500);
    Y = map(yData, -130, 130, -1500, 1500);
    // Get current time
    current_time = millis();
    delta_time = (current_time - last_time) / 1000.0; // Time in seconds
    last_time = current_time;
    

    // Example: Send current command for 10 A (scaled to 16384 range)
    if (millis() - lastSend >= sendInterval) {
        byte data[8] = {0};
        if (mode == 0){
            axis_logic(X, Y);
        }         
        else{
            axis_logic2(X, Y);
        }
        data[0] = (LB >> 8) & 0xFF;     // LF HIGH
        data[1] = LB & 0xFF;            // LF LOW
        data[2] = (LF >> 8) & 0xFF;     // LB HIGH
        data[3] = LF & 0xFF;            // LB LOW
        data[4] = (RB >> 8) & 0xFF;     // RF HIGH
        data[5] = RB & 0xFF;            // RF LOW
        data[6] = (RF >> 8) & 0xFF;     // RB HIGH
        data[7] = RF & 0xFF;            // RB LOW

        // Send CAN message
        if (CAN.sendMsgBuf(sendID, 0, 8, data) == CAN_OK) {
            // Serial.print("Command sent.......");
        } else {
            Serial.println("Failed to send command");
        }
        // Example: 10 A for Motor 1
        lastSend = millis();
    }

    // Check for feedback
    if (!digitalRead(CAN_INT)) {
        if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
            if (rxId == feedbackID_LF) {

                // Extract feedback data
                int16_t rotor_angle = (rxBuf[0] << 8) | rxBuf[1];
                int16_t rotational_speed = (rxBuf[2] << 8) | rxBuf[3];
                int16_t torque_current = (rxBuf[4] << 8) | rxBuf[5];
                int8_t temperature = rxBuf[6];

                // Print feedback data
                // Serial.print("x: ");
                // Serial.print(xData);
                // Serial.print(", y: ");
                // Serial.println(yData);
                // Serial.print(", (RPM): ");
                // Serial.print(rotational_speed);
                // Serial.print(", Torque(mA): ");
                // Serial.print(torque_current);
                // Serial.print(", (°C): ");
                // Serial.println(temperature);
            }
        }

    }
}
