#ifndef CONSTANTS_H
#define CONSTANTS_H

#define MIN 0
#define MAX 255

// Arduino pin defines
const byte MOTOR_LEFT_IN1 = 3;
const byte MOTOR_LEFT_IN2 = 4;
// Right Motor runs backwards so swap the pin numbers
const byte MOTOR_RIGHT_IN1 = 6;
const byte MOTOR_RIGHT_IN2 = 5;
const byte FRONT_TOF_SHUTDOWN = 7;  //Active Low

// Defines for IMU
#define WIRE_PORT Wire
#define AD0_VAL 0

// Defines for TOF sensors
const byte SIDE_TOF_ADDRESS = 0x54;

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "fbaf56fb-a094-4bf5-b5b9-71571ba584a7"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

#endif