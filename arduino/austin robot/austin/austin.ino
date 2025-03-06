#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include "math.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "fbaf56fb-a094-4bf5-b5b9-71571ba584a7"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

// //////////// BLE UUIDs ////////////
// #define BLE_UUID_TEST_SERVICE "4f347f90-757a-4329-959b-03378e373c24"
// #define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
// #define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
// #define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
// //////////// BLE UUIDs ////////////

//#define USE_SPI
#define SERIAL_PORT Serial
#define SPI_PORT SPI
#define CS_PIN 2
#define WIRE_PORT Wire
#define AD0_VAL 0

#ifdef USE_SPI
ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN_1 8
#define INTERRUPT_PIN_1 3

//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN_1, INTERRUPT_PIN_1);
SFEVL53L1X distanceSensor2;

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

int j = 0;
int i = 0;
int n = 0;
int z = 0;
// float array[500];
// float temp_array[500];
// float sec_array[500];


// double pitch_acc[400];
// double roll_acc[400];
// double pitch_acc_lpf[400];
// double roll_acc_lpf[400];

float period = 0.0;
float alpha = 0.0;

// double pitch_gyro[400];
// double roll_gyro[400];

#define arraySize 5000

int data_size = arraySize;

float time_array[arraySize];

float speed_array[arraySize];

double yaw_gyro_array[arraySize];

// double pitch_comp[400];
// double roll_comp[400];
// double yaw_comp[400];

float dt = 0.0;

float time_start = 0.0;

double TOF_1[arraySize];
double TOF_2[arraySize];

float Front_TOF = 0.0;
float Past_TOF = 0.0;

float KP = 0.0;
float KI = 0.0;
float KD = 0.0;

float KP_Orient = 0.0;
float KI_Orient = 0.0;
float KD_Orient = 0.0;

float P = 0.0;
float I = 0.0;
float D = 0.0;

float P_Orient = 0.0;
float I_Orient = 0.0;
float D_Orient = 0.0;

float dist_PID = 0.0;
float map_current_dis = 0.0;

float PID = 0.0;
float PID_Orient = 0.0;

int Right_F = 9;
int Right_B = 11;
int Left_F = 13;
int Left_B = 12;

float RF_Speed = 0.0;
float RB_Speed = 0.0;
float LF_Speed = 0.0;
float LB_Speed = 0.0;

float Right_Speed = 0.0;
float Left_Speed = 0.0;

float dis_error = 0.0;
float prev_error = 0.0;
float sum_error = 0.0;
float current_time = 0.0;
float past_time = 0.0;
float PID_current_time = 0.0;
float PID_past_time = 0.0;
float PID_dt = 0.0;
float distance_interp = 0.0;

float distance_goal = 0.0;
float angle_goal = 0.0;
float angle_error = 0.0;

float P_Array[arraySize];
float I_Array[arraySize];
float D_Array[arraySize];
float PID_Array[arraySize];

float PID_dt_Array[arraySize];
float dt_Array[arraySize];

int PID_On = 0;
int ORIENT_On = 0;
int Store_On = 0;
int DRIVE_On = 0;
int STUNT_On = 0;

float yaw_gyro = 0.0;
float yaw_gyro_past = 0.0;

float current_speed = 0.0;

float Orient_Start_Time = 0.0;
//////////// Global Variables ////////////


enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    GET_TIME_MILLIS,
    GET_TIME_MILLIS_LOOP,
    STORE_TIME_DATA,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    FREQ_SPEC,
    ACC_LPF,
    GYRO,
    GYRO_LPF,
    IMU_DATA,
    DATA_RATE,
    DANCE,
    SET_VEL,
    TOF,
    LINEAR_SET_PID,
    START_PID,
    ORIENT_SET_PID,
    START_ORIENT,
    BRAKE,
    SEND_PID,
    DRIVE,
    SEND_STORE,
    STUNT,
    SET_ANG,
    SET_DIS,
    MAP,
};

void handle_command() {   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * GET_TIME_MILLIS
         */
        case GET_TIME_MILLIS:
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        
        /*
         * GET_TIME_MILLIS_LOOP
         */
        case GET_TIME_MILLIS_LOOP:
            i = 0;
            long init_t;
            init_t = millis();
            while (millis()-init_t < 3000 && i < 500){
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
            }
            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;

        /*
         * STORE_TIME_DATA
         */
        case STORE_TIME_DATA:
            for(i = 0; i < 500; i++){
              //array[i] = (int)millis();
             }
            break;
        
        /*
         * SEND_TIME_DATA
         */
        case SEND_TIME_DATA:
            for(j = 0; j < 500; j++){
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              // tx_estring_value.append(array[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            
            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        
        /*
         * GET_TEMP_READINGS
         */
        case GET_TEMP_READINGS:
            for(i = 0; i < 500; i++){
              // sec_array[i] = (int)millis();
              // temp_array[i] = (float)getTempDegC();
             }
            for(j = 0; j < 500; j++){
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              // tx_estring_value.append(sec_array[j]);
              tx_estring_value.append("|C:");
              // tx_estring_value.append(temp_array[j]);
              tx_estring_value.append("|");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            
            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;

        /*
         * FREQ_SPEC
         */
        case FREQ_SPEC:
            if (myICM.dataReady()) {
              for(i = 0; i < data_size; i++){
                myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
                time_array[i] = ((float)millis())/1000.0;
                // pitch_acc[i] = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                // roll_acc[i] = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                delay(30);
              }
              for(j = 0; j < data_size; j++){
                tx_estring_value.clear();
                tx_estring_value.append("Time:");
                tx_estring_value.append(time_array[j]);
                tx_estring_value.append("|Pitch:");
                // tx_estring_value.append(pitch_acc[j]);
                tx_estring_value.append("|Roll:");
                // tx_estring_value.append(roll_acc[j]);
                tx_estring_value.append("|");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
              }
            } else {
              SERIAL_PORT.println("Waiting for data");
              delay(500);
            }
            break;

        /*
         * ACC_LPF
         */
        case ACC_LPF:
            if (myICM.dataReady()) {
              // pitch_acc_lpf[0] = 0;
              // roll_acc_lpf[0] = 0;
              for(i = 0; i < data_size; i++){
                myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
                time_array[i] = (float)millis();
                // pitch_acc[i] = atan2(myICM.accX(), myICM.accZ()) * 180.0 / M_PI;
                // roll_acc[i] = atan2(myICM.accY(), myICM.accZ()) * 180.0 / M_PI;
                delay(30);
              }
              for(i = 1; i < data_size; i++){
                period = (time_array[i]-time_array[i-1]);
                alpha = period/(period + (1/(2*M_PI*5)));

                // pitch_acc_lpf[i] = (alpha*pitch_acc[i]) + ((1-alpha)*pitch_acc_lpf[i-1]);
                // roll_acc_lpf[i] = (alpha*roll_acc[i]) + ((1-alpha)*roll_acc_lpf[i-1]);
                // pitch_acc_lpf[i-1] = pitch_acc_lpf[i];
                // roll_acc_lpf[i-1] = roll_acc_lpf[i];
              }
              // for(j = 0; j < data_size; j++){
              //   tx_estring_value.clear();
              //   tx_estring_value.append("Time:");
              //   tx_estring_value.append(time_array[j]);
              //   tx_estring_value.append("|Pitch:");
              //   // tx_estring_value.append(pitch_acc_lpf[j]);
              //   tx_estring_value.append("|Roll:");
              //   // tx_estring_value.append(roll_acc_lpf[j]);
              //   tx_estring_value.append("|");
              //   tx_characteristic_string.writeValue(tx_estring_value.c_str());
              // }
            } else {
              SERIAL_PORT.println("Waiting for data");
              delay(500);
            }
            break;

        /*
         * GYRO
         */
        case GYRO:
            if (myICM.dataReady()) {
              // pitch_gyro[0] = 0;
              // roll_gyro[0] = 0;
              // yaw_gyro[0] = 0;
              for(i = 1; i < data_size; i++){
                time_array[i] = (float)millis();
                myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
                // pitch_gyro[i] = pitch_gyro[i-1] + myICM.gyrY()*.05;
                // roll_gyro[i] = roll_gyro[i-1] + myICM.gyrX()*.05;
                // yaw_gyro[i] = yaw_gyro[i-1] + myICM.gyrZ()*.05;
                delay(50);
              }
              // for(j = 0; j < data_size; j++){
              //   tx_estring_value.clear();
              //   tx_estring_value.append("Time:");
              //   tx_estring_value.append(time_array[j]);
              //   tx_estring_value.append("|Pitch:");
              //   // tx_estring_value.append(pitch_gyro[j]);
              //   tx_estring_value.append("|Roll:");
              //   // tx_estring_value.append(roll_gyro[j]);
              //   tx_estring_value.append("|Yaw:");
              //   // tx_estring_value.append(yaw_gyro[j]);
              //   tx_estring_value.append("|");
              //   tx_characteristic_string.writeValue(tx_estring_value.c_str());
              // }
            } else {
              SERIAL_PORT.println("Waiting for data");
              delay(500);
            }
            break;
        
        /*
         * GYRO_LPF
         */
        case GYRO_LPF:
            if (myICM.dataReady()) {
              // pitch_comp[0] = 0;
              // roll_comp[0] = 0;
              // yaw_comp[0] = 0;
              // pitch_gyro[0] = 0;
              // roll_gyro[0] = 0;
              // yaw_gyro[0] = 0;
              // pitch_acc_lpf[0] = 0;
              // roll_acc_lpf[0] = 0;
              time_array[0] = (float)millis();
              
              for(i = 1; i < data_size; i++){
                time_array[i] = (float)millis();
                dt = (time_array[i] - time_array[i-1])/1000.0;
              
                myICM.getAGMT();  // The values are only updated when you call 'getAGMT'

                // pitch_acc[i] = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                // roll_acc[i] = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                // pitch_gyro[i] = pitch_gyro[i-1] + myICM.gyrY()*dt;
                // roll_gyro[i] = roll_gyro[i-1] + myICM.gyrX()*dt;
                // yaw_gyro[i] = yaw_gyro[i-1] + myICM.gyrZ()*dt;

                delay(30);
              }
              for(i = 1; i < data_size; i++){
                period = (time_array[i]-time_array[i-1]);
                alpha = period/(period + (1/(2*M_PI*5)));

                // pitch_acc_lpf[i] = (alpha*pitch_acc[i]) + ((1-alpha)*pitch_acc_lpf[i-1]);
                // roll_acc_lpf[i] = (alpha*roll_acc[i]) + ((1-alpha)*roll_acc_lpf[i-1]);
                // pitch_acc_lpf[i-1] = pitch_acc_lpf[i];
                // roll_acc_lpf[i-1] = roll_acc_lpf[i];
              }
              for(i = 1; i < data_size; i++){
                period = (time_array[i]-time_array[i-1]);
                alpha = period/(period + (1/(2*M_PI*5)));

                // pitch_comp[i] = (pitch_comp[i-1]+pitch_gyro[i])*(1-alpha) + (pitch_acc_lpf[i]*alpha);
                // roll_comp[i] = (roll_comp[i-1]+roll_gyro[i])*(1-alpha) + (roll_acc_lpf[i]*alpha);
                // yaw_comp[i] = yaw_comp[i] + yaw_gyro[i];
              }
              // for(j = 0; j < data_size; j++){
              //   tx_estring_value.clear();
              //   tx_estring_value.append("Time:");
              //   tx_estring_value.append(time_array[j]);
              //   tx_estring_value.append("|Pitch:");
              //   // tx_estring_value.append(pitch_comp[j]);
              //   tx_estring_value.append("|Roll:");
              //   // tx_estring_value.append(roll_comp[j]);
              //   tx_estring_value.append("|Yaw:");
              //   // tx_estring_value.append(yaw_comp[j]);
              //   tx_estring_value.append("|");
              //   tx_characteristic_string.writeValue(tx_estring_value.c_str());
              // }
            } else {
                SERIAL_PORT.println("Waiting for data");
                delay(500);
              }
            break;

        /*
         * IMU_DATA
         */
        case IMU_DATA:
            // pitch_comp[0] = 0;
            // roll_comp[0] = 0;
            // yaw_comp[0] = 0;
            // pitch_gyro[0] = 0;
            // roll_gyro[0] = 0;
            // yaw_gyro[0] = 0;
            // pitch_acc_lpf[0] = 0;
            // roll_acc_lpf[0] = 0;
            time_start = (float)millis();
            time_array[0] = time_start;

            while(j < data_size and (float)millis()-time_start < 6000){
              if (myICM.dataReady()) {
                time_array[j] = (float)millis();
                dt = (time_array[j] - time_array[j-1])/1000.0;
              
                myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
                
                // pitch_acc[j] = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                // roll_acc[j] = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                // pitch_gyro[j] = pitch_gyro[j-1] + myICM.gyrY()*dt;
                // roll_gyro[j] = roll_gyro[j-1] + myICM.gyrX()*dt;
                // yaw_gyro[j] = yaw_gyro[j-1] + myICM.gyrZ()*dt;
                j++;
              }
            }
              for(i = 1; i < data_size; i++){
                period = (time_array[i]-time_array[i-1]);
                alpha = period/(period + (1/(2*M_PI*5)));

                // pitch_acc_lpf[i] = (alpha*pitch_acc[i]) + ((1-alpha)*pitch_acc_lpf[i-1]);
                // roll_acc_lpf[i] = (alpha*roll_acc[i]) + ((1-alpha)*roll_acc_lpf[i-1]);
                // pitch_acc_lpf[i-1] = pitch_acc_lpf[i];
                // roll_acc_lpf[i-1] = roll_acc_lpf[i];
              }
              for(i = 1; i < data_size; i++){
                period = (time_array[i]-time_array[i-1]);
                alpha = period/(period + (1/(2*M_PI*5)));

                // pitch_comp[i] = (pitch_comp[i-1]+pitch_gyro[i])*(1-alpha) + (pitch_acc_lpf[i]*alpha);
                // roll_comp[i] = (roll_comp[i-1]+roll_gyro[i])*(1-alpha) + (roll_acc_lpf[i]*alpha);
                // yaw_comp[i] = yaw_comp[i] + yaw_gyro[i];
              }
              for(j = 0; j < data_size; j++){
                tx_estring_value.clear();
                tx_estring_value.append("Time:");
                tx_estring_value.append(time_array[j]);
                tx_estring_value.append("|Pitch:");
                // tx_estring_value.append(pitch_comp[j]);
                tx_estring_value.append("|Roll:");
                // tx_estring_value.append(roll_comp[j]);
                tx_estring_value.append("|Yaw:");
                // tx_estring_value.append(yaw_comp[j]);
                tx_estring_value.append("|");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
              }
            break;

        /*
         * DATA_RATE
         */
        case DATA_RATE:
            char char_data[MAX_MSG_SIZE];

            success = robot_cmd.get_next_value(char_data);
            if (!success){
                return;
            }

            tx_estring_value.clear();
            tx_estring_value.append(char_data);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:
            Serial.println("SET_VEL Activated");

            break;
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
        /*
         * TOF
         */
        case TOF:

          //TOFCollect();

          for(j = 0; j < data_size; j++){
            tx_estring_value.clear();
            tx_estring_value.append("Time:");
            tx_estring_value.append(time_array[j]);
            tx_estring_value.append("|TOF 1:");
            tx_estring_value.append(TOF_1[j]);
            tx_estring_value.append("|TOF 2:");
            tx_estring_value.append(TOF_2[j]);
            tx_estring_value.append("|");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }

          break;

        /*
         * LINEAR_SET_PID
         */
        case LINEAR_SET_PID:
          float KP_New;
          float KI_New;
          float KD_New;

          success = robot_cmd.get_next_value(KP_New);
            if (!success){
                return;
            }
          
          success = robot_cmd.get_next_value(KI_New);
            if (!success){
                return;
            }
          
          success = robot_cmd.get_next_value(KD_New);
            if (!success){
                return;
            }
          
          KP = KP_New;
          KI = KI_New;
          KD = KD_New;

          break;

        /*
         * START_PID
         */
        case START_PID:
          float START;
          float dis_goal;
          float STORE;

          success = robot_cmd.get_next_value(START);
            if (!success){
                return;
            }
          success = robot_cmd.get_next_value(dis_goal);
            if (!success){
                return;
            }
          success = robot_cmd.get_next_value(STORE);
            if (!success){
                return;
            }
          
          distance_goal = dis_goal;
          
          if (START == 1) {
            sum_error = 0.0;
            PID = 0.0;
            current_time = (float)millis();
            past_time = current_time;
            PID_current_time = (float)millis();
            PID_past_time = PID_current_time;
            PID_On = 0;
            distanceSensor2.startRanging();
          }
          else{
            PID_On = 0;
            Brake();
          }
          if (STORE == 1){
            Store_On = true;
          }
        break;

        /*
         * ORIENT_SET_OID
         */
        case ORIENT_SET_PID:
          float KP_Orient_New;
          float KI_Orient_New;
          float KD_Orient_New;

          success = robot_cmd.get_next_value(KP_Orient_New);
            if (!success){
                return;
            }
          
          success = robot_cmd.get_next_value(KI_Orient_New);
            if (!success){
                return;
            }
          
          success = robot_cmd.get_next_value(KD_Orient_New);
            if (!success){
                return;
            }
          
          KP_Orient = KP_Orient_New;
          KI_Orient = KI_Orient_New;
          KD_Orient = KD_Orient_New;

        break;

        /*
         * START_ORIENT
         */
        case START_ORIENT:
          float START_O;
          float ang_goal;
          float STORE_O;

          success = robot_cmd.get_next_value(START_O);
            if (!success){
                return;
            }
          success = robot_cmd.get_next_value(ang_goal);
            if (!success){
                return;
            }
          success = robot_cmd.get_next_value(STORE_O);
            if (!success){
                return;
            }
          
          angle_goal = ang_goal;
          
          if (START_O == 1){
            yaw_gyro = 0.0;
            dt = 0.0;
            PID_dt = 0.0;
            sum_error = 0.0;
            angle_error = 0.0;
            PID_Orient = 0.0;
            current_time = (float)millis();
            past_time = current_time;
            PID_current_time = (float)millis();
            PID_past_time = PID_current_time;
            ORIENT_On = true;
            distanceSensor1.startRanging();
            distanceSensor2.startRanging();
          }
          else{
            ORIENT_On = false;
            Brake();
          }
          if (STORE_O == 1){
            Store_On = true;
          }
        break;
      
        /*
         * BRAKE
         */
        case BRAKE:

          Brake();
          PID_On = 0;
          ORIENT_On = 0;
          DRIVE_On = 0;

        break;

        /*
         * SEND_PID
         */
        case SEND_PID:
        tx_estring_value.clear();
          for(int send_ind = 0; send_ind < j; send_ind++){
            tx_estring_value.append("Time:");
            tx_estring_value.append(time_array[send_ind]);
            tx_estring_value.append("|TOF 1:");
            tx_estring_value.append(TOF_1[send_ind]);
            tx_estring_value.append("|TOF 2:");
            tx_estring_value.append(TOF_2[send_ind]);
            tx_estring_value.append("|P:");
            tx_estring_value.append(P_Array[send_ind]);
            tx_estring_value.append("|I:");
            tx_estring_value.append(I_Array[send_ind]);
            tx_estring_value.append("|D:");
            tx_estring_value.append(D_Array[send_ind]);
            tx_estring_value.append("|PID:");
            tx_estring_value.append(PID_Array[send_ind]);
            tx_estring_value.append("|Yaw:");
            tx_estring_value.append(yaw_gyro_array[send_ind]);
            tx_estring_value.append("|");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            tx_estring_value.clear();
          }
          break;

          /*
          * DRIVE
          */
          case DRIVE:
            int DRIVE_START;

            success = robot_cmd.get_next_value(DRIVE_START);
            if (!success){
                return;
            }

            if (DRIVE_START == 1){
              z = 1;
              current_time = (float)millis();
              ORIENT_On = 0;
              delay(1000);
              DRIVE_On = 1;
            }
            else{
              DRIVE_On = 0;
              Brake();
            }

          break;

          /*
          * SEND_STORE
          */
          case SEND_STORE:
            tx_estring_value.clear();
            for(int send_ind = 0; send_ind < j; send_ind++){
              tx_estring_value.append("Time:");
              tx_estring_value.append(time_array[send_ind]);
              tx_estring_value.append("|TOF 2:");
              tx_estring_value.append(TOF_2[send_ind]);
              tx_estring_value.append("|Speed:");
              tx_estring_value.append(speed_array[send_ind]);
              tx_estring_value.append("|");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
              tx_estring_value.clear();
            }
          break;

          /*
          * STUNT
          */
          case STUNT:
            int STUNT_ON;
            int STORE_VAL;

            success = robot_cmd.get_next_value(STUNT_ON);
            if (!success){
                return;
            }

            success = robot_cmd.get_next_value(STORE_VAL);
            if (!success){
                return;
            }

            angle_goal = 0.0;

            if (STUNT_ON == 1) {
              sum_error = 0.0;
              PID = 0.0;
              PID_Orient = 0.0;
              current_time = (float)millis();
              past_time = current_time;
              PID_current_time = (float)millis();
              PID_past_time = PID_current_time;
              STUNT_On = true;
              DRIVE_On = true;
              distanceSensor2.startRanging();
              delay(1000);
            }
            else{
              ORIENT_On = false;
              STUNT_On = false;
              DRIVE_On = 0;
              Brake();
            }
            if (STORE_VAL == 1){
              Store_On = true;
            }
          break;

          /*
          * SET_ANG
          */
          case SET_ANG:
            float re_ang_goal;

            success = robot_cmd.get_next_value(re_ang_goal);
            if (!success){
                return;
            }
            sum_error = 0.0;
            angle_error = 0.0;
            DRIVE_On = 0;
            ORIENT_On = 1;
            PID = 0;
            PID_Orient = 0.0;
            angle_goal = re_ang_goal;

          break;

          /*
          * SET_DIS
          */
          case SET_DIS:
            float re_dis_goal;

            success = robot_cmd.get_next_value(re_dis_goal);
            if (!success){
                return;
            }
            PID_On = 1;
            ORIENT_On = 0;
            map_current_dis = (float)distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            distance_goal = map_current_dis - re_dis_goal;

          break;

          /*
          * MAP
          */
          case MAP:
            int MAP_ON;
            int STORE_MAP;

            success = robot_cmd.get_next_value(MAP_ON);
            if (!success){
                return;
            }

            success = robot_cmd.get_next_value(STORE_MAP);
            if (!success){
                return;
            }

            if (MAP_ON == 1) {
              yaw_gyro = 0.0;
              dt = 0.0;
              PID_dt = 0.0;
              sum_error = 0.0;
              PID = 0.0;
              PID_Orient = 0.0;
              current_time = (float)millis();
              past_time = current_time;
              PID_current_time = (float)millis();
              PID_past_time = PID_current_time;
              ORIENT_On = 0;
              PID_On = 0;
              distanceSensor2.startRanging();
              delay(500);
            }
            else{
              ORIENT_On = 0;
              PID_On = 0;
              DRIVE_On = 0;
              Brake();
            }
            if (STORE_MAP == 1){
              Store_On = 1;
            }
            else{
              Store_On = 0;
            }
          break;
}
}

void
setup()
{
    analogWrite(Right_F, 0);
    analogWrite(Right_B, 0);
    analogWrite(Left_F, 0);
    analogWrite(Left_B, 0);

    Serial.begin(115200);

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

    pinMode(LED_BUILTIN, OUTPUT);

    analogWrite(Right_F, 0);
    analogWrite(Right_B, 0);
    analogWrite(Left_F, 0);
    analogWrite(Left_B, 0);

    Wire.begin();

    analogWrite(Right_F, 0);
    analogWrite(Right_B, 0);
    analogWrite(Left_F, 0);
    analogWrite(Left_B, 0);

    Serial.println("VL53L1X Qwiic Test");
    pinMode(SHUTDOWN_PIN_1, OUTPUT);
    digitalWrite(SHUTDOWN_PIN_1, LOW);
    distanceSensor1.setI2CAddress(0x20);
    digitalWrite(SHUTDOWN_PIN_1, HIGH);

    if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor 1 online!");

    if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
      while (1)
       ;
    }
    Serial.println("Sensor 2 online!");

    distanceSensor1.setDistanceModeLong();
    distanceSensor2.setDistanceModeLong();

  int i;
  if (i < 3) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    i = i + 1;
  }

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized) {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm2;
    myFSS.g = dps1000;
    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.print(F("setFullScale returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }
  }
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();
    write_data();

    if (central) {
        Brake();
        Serial.print("Connected to: ");
        Serial.println(central.address());
        j = 0;
        while (central.connected()) {
          read_data();
          if (PID_On && j < data_size){
            PID_current_time = (float)millis();
            PID_dt = (PID_current_time-PID_past_time)/1000.0;
            PID_past_time = PID_current_time;
            if (distanceSensor2.checkForDataReady()){
              current_time = (float)millis();
              dt = (current_time-past_time)/1000.0;
              past_time = current_time;
              Past_TOF = Front_TOF;
              Front_TOF = (float)distanceSensor2.getDistance();
              distanceSensor2.clearInterrupt();
              dist_PID = Front_TOF;
            }
            else if (Past_TOF > 0){
              distance_interp = (((Front_TOF-Past_TOF)/dt)*PID_dt)+dist_PID;
              dist_PID = distance_interp;
            }
            Linear_PIDControl();
            Left_Speed = -PID*0.75;
            Right_Speed = -PID*0.95;
            RightWheel();
            LeftWheel();
            if (Store_On && j < data_size){
              PIDStore();
            }
            j++;
          }
          if (ORIENT_On && j < data_size){
            PID_current_time = (float)millis();
            PID_dt = (PID_current_time-PID_past_time)/1000.0;
            PID_past_time = PID_current_time;
            if (myICM.dataReady()){
              if (distanceSensor2.checkForDataReady()){
                Front_TOF = (float)distanceSensor2.getDistance();
                distanceSensor2.clearInterrupt();
                dist_PID = Front_TOF;
              }
              //else{
                //dist_PID = 0.0;
              //}
              current_time = (float)millis();
              dt = (current_time-past_time)/1000.0;
              past_time = current_time;
              yaw_gyro_past = yaw_gyro;
              myICM.getAGMT();
              yaw_gyro = yaw_gyro_past + myICM.gyrZ()*dt;
              Orient_PIDControl();
              Left_Speed = PID_Orient;
              Right_Speed = -PID_Orient;

              RightWheel();
              LeftWheel();
              if (Store_On && j < data_size){
                PIDStore_O();
              }
              j++;
            }
            // if (current_time - Orient_Start_Time > 600){
            //   DRIVE_On = 1;
            //   ORIENT_On = false;
            // }
          }
          if (DRIVE_On == 1){
            // if (distanceSensor2.checkForDataReady()){
                current_time = (float)millis();
                dt = (current_time-past_time)/1000.0;
                past_time = current_time;
                Past_TOF = Front_TOF;
                Front_TOF = (float)distanceSensor2.getDistance();
                distanceSensor2.clearInterrupt();
                dist_PID = Front_TOF;
                if (Store_On && j < data_size){
                   PIDStore_O();
                 }
                Left_Speed = -50;
                Right_Speed = -61;
                RightWheel();
                LeftWheel();
                j++;
            // else if(Past_TOF > 0){
            //   distance_interp = (((Front_TOF-Past_TOF)/dt)*PID_dt)+dist_PID;
            //   dist_PID = distance_interp;
            // }
          }
          if (STUNT_On && j < data_size){
            if (dist_PID < 1600.0){
              Orient_Start_Time = (float)millis();
              DRIVE_On = 0;
              ORIENT_On = true;
              angle_goal = 180.0;
              STUNT_On = false;
            }
          }
        }
    }
    else{
      Serial.println("Disconnected");
      Brake();
    }
    
}

// void TOFCollect(){
//   time_start = (float)millis();
//   time_array[0] = time_start;
//   distanceSensor1.setDistanceModeLong();
//   distanceSensor2.setDistanceModeLong();
//   distanceSensor1.startRanging();
//   distanceSensor2.startRanging();

//   while(n < data_size and (float)millis()-time_start < 6000){
//     if (distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady()){
//       time_array[n] = (float)millis();
              
//       int distance_1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
//       TOF_1[n] = distance_1;

//       int distance_2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
//       TOF_2[n] = distance_2;
      
//       n++;
//     }
//   }
//   distanceSensor1.clearInterrupt();
//   distanceSensor2.clearInterrupt();
//   distanceSensor1.stopRanging();
//   distanceSensor2.stopRanging();
// }

void LeftWheel(){
  if (Left_Speed > 0.0 || Left_Speed == 0.0){
    LF_Speed = 255.0*(Left_Speed)/100.0;
    if (LF_Speed < 40){
      LF_Speed = 40;
    }
    LB_Speed = 0.0;
  }
  if (Left_Speed < 0.0){
    LF_Speed = 0.0;
    LB_Speed = -255.0*(Left_Speed)/100.0;
    if (LB_Speed < 40){
      LB_Speed = 40;
    }
  }
  analogWrite(Left_F, LF_Speed);
  analogWrite(Left_B, LB_Speed);
}

void RightWheel(){
  if (Right_Speed > 0.0 || Right_Speed == 0.0){
    RF_Speed = 255*(Right_Speed)/100;
    if (RF_Speed < 40){
      RF_Speed = 40;
    }
    RB_Speed = 0.0;
  }
  if (Right_Speed < 0.0){
    RF_Speed = 0.0;
    RB_Speed = -255.0*(Right_Speed)/100.0;
    if (RB_Speed < 40){
      RB_Speed = 40;
    }
  }
  analogWrite(Right_F, RF_Speed);
  analogWrite(Right_B, RB_Speed);
}

void Brake(){
  analogWrite(Right_F, 255);
  analogWrite(Right_B, 255);
  analogWrite(Left_F, 255);
  analogWrite(Left_B, 255);
}

void Linear_PIDControl(){
  prev_error = dis_error;
  dis_error = dist_PID - distance_goal;
  sum_error = sum_error+(dis_error*dt);

  P = KP*dis_error;
  I = KI*sum_error;
  D = KD*(dis_error-prev_error)/dt;

  if (I > 100.0){
    I = 100.0;
  }
  if (I < -100.0){
    I = -100.0;
  }

  PID = P + I + D;

  if (PID > 100.0){
    PID = 100.0;
  }
  if (PID < -100.0){
    PID = -100.0;
  }
}

void Orient_PIDControl(){
  prev_error = angle_error;
  angle_error = yaw_gyro - angle_goal;
  sum_error = sum_error+(angle_error*dt);

  P_Orient = KP_Orient*angle_error;
  I_Orient = KI_Orient*sum_error;
  D_Orient = KD_Orient*(angle_error-prev_error)/dt;

  if (I_Orient > 100.0){
    I_Orient = 100.0;
  }
  if (I_Orient < -100.0){
    I_Orient = -100.0;
  }

  PID_Orient = P_Orient + I_Orient + D_Orient;

  if (PID_Orient > 100.0){
    PID_Orient = 100.0;
  }
  if (PID_Orient < -100.0){
    PID_Orient = -100.0;
  }
}

void PIDStore(){
    time_array[j] = PID_current_time;
    P_Array[j] = P;
    I_Array[j] = I;
    D_Array[j] = D;
    PID_Array[j] = PID;
    TOF_2[j] = dist_PID;
    dt_Array[j] = dt;
    PID_dt_Array[j] = PID_dt;
    yaw_gyro_array[j] = yaw_gyro;
}

void PIDStore_O(){
    time_array[j] = PID_current_time;
    P_Array[j] = P_Orient;
    I_Array[j] = I_Orient;
    D_Array[j] = D_Orient;
    PID_Array[j] = PID_Orient;
    TOF_2[j] = dist_PID;
    dt_Array[j] = dt;
    PID_dt_Array[j] = PID_dt;
    yaw_gyro_array[j] = yaw_gyro;
}

void Store(){
  time_array[j] = current_time;
  TOF_2[j] = dist_PID;
  speed_array[j] = current_speed;
}