#include "/Users/karlt/Documents/GitHub/fastrobots/arduino/BLECStringCharacteristic.h"
#include "/Users/karlt/Documents/GitHub/fastrobots/arduino/EString.h"
#include "/Users/karlt/Documents/GitHub/fastrobots/arduino/RobotCommand.h"
#include <ArduinoBLE.h>
#include <ICM_20948.h>
#include <math.h>

// Defines for IMU
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 0

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "fbaf56fb-a094-4bf5-b5b9-71571ba584a7"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

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

//////////// Global Variables ////////////
int i;
long time_start, time_stamp, time_prev, cnt, period;
float dt, alpha;
#define data_size 2000
int time_array[data_size];
float temp_array[data_size];
float pitch_a[data_size], roll_a[data_size];
float pitch_a_lpf[data_size], roll_a_lpf[data_size];
float pitch_g[data_size], roll_g[data_size], yaw_g[data_size];
float pitch_g_lpf[data_size], roll_g_lpf[data_size], yaw_g_lpf[data_size];
bool imu_ready;

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_TIME_MILLIS_LOOP,
    STORE_TIME_DATA,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    GET_IMU_DATA
};

// Create an I2C object for the IMU
ICM_20948_I2C myICM;

void
blink3()
{
  for (i=0; i<3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(1000);                      // wait for a second
  }
}

void
handle_command()
{   
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

            /*
             * Your code goes here
             */
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            // Serial.print("Sent back: ");
            // Serial.println(tx_estring_value.c_str());

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

            break;

        case GET_TIME_MILLIS:

            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            // Serial.print("Sent back: ");
            // Serial.println(tx_estring_value.c_str());

            break;
        
        case GET_TIME_MILLIS_LOOP:

            time_start = millis();
            cnt = 0;
            while (millis() - time_start < 2000) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)millis());
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
              cnt = cnt + 1;
            }

            Serial.print("Total count: ");
            Serial.println(cnt);

            break;

        case STORE_TIME_DATA:
            for (i=0; i<500; i++) {
              time_array[i] = millis();
              temp_array[i] = getTempDegC();
            }
            Serial.println("Stored 500 time samples");
            break;

        case SEND_TIME_DATA:
            for (i=0; i<500; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append(time_array[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            Serial.println("Sent 500 time samples");
            break;

        case GET_TEMP_READINGS:
            for (i=0; i<500; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append(time_array[i]);
              tx_estring_value.append(",C:");
              tx_estring_value.append(temp_array[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            Serial.println("Sent 500 time and temp samples");
            break;

        case GET_IMU_DATA:

            time_start = millis();

            i = 0;

            while (millis() - time_start < 5000)
            {
              if (myICM.dataReady())
              {
                time_array[i] = millis();
                period = time_array[i]-time_array[i-1];
                dt = period/1000.0;
                alpha = period/(period+(1/(2*M_PI*5)));

                myICM.getAGMT();            // The values are only updated when you call 'getAGMT'
              
              // Get the IMU accelerometer data

                pitch_a[i] = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                roll_a[i]  = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;

              // process the accel data through a complimentary filter

                if (i>0) {
                  pitch_a_lpf[i] = alpha*pitch_a[i] + (1-alpha)*(pitch_a[i-1]);
                  roll_a_lpf[i]  = alpha*roll_a[i]  + (1-alpha)*(roll_a[i-1]);
                }
                else {
                  pitch_a_lpf[i] = pitch_a[i];
                  roll_a_lpf[i]  = roll_a[i];
                }

/*
                if (pitch_a[i] < -90) {
                  pitch_a[i] = pitch_a[i] + 180;
                }
                if (pitch_a[i] > 90) {
                  pitch_a[i] = pitch_a[i] - 180;
                }
                if (roll_a[i] < -90) {
                  roll_a[i] = roll_a[i] + 180;
                }
                if (roll_a[i] > 90) {
                  roll_a[i] = roll_a[i] - 180;
                }
*/

              // Get the IMU gyro data

                if (i>0) {
                  roll_g[i]  = roll_g[i-1]  + myICM.gyrX()*dt;
                  pitch_g[i] = pitch_g[i-1] + myICM.gyrY()*dt;
                  yaw_g[i]   = yaw_g[i-1]   + myICM.gyrZ()*dt;
                }
                else {
                  roll_g[i]  = myICM.gyrX()*dt;
                  pitch_g[i] = myICM.gyrY()*dt;
                  yaw_g[i]   = myICM.gyrZ()*dt;
                }

                if (i>0) {
                  roll_g_lpf[i]  = (roll_g_lpf[i-1]  + roll_g[i])*(1-alpha)  + roll_a_lpf[i]*alpha;
                  pitch_g_lpf[i] = (pitch_g_lpf[i-1] + pitch_g[i])*(1-alpha) + pitch_a_lpf[i]*alpha;
                }
                else {
                  roll_g_lpf[i]  = (roll_g[i])*(1-alpha)  + roll_a_lpf[i]*alpha;
                  pitch_g_lpf[i] = (pitch_g[i])*(1-alpha) + pitch_a_lpf[i]*alpha;
                }
                yaw_g_lpf[i]   =  yaw_g_lpf[i]   + yaw_g[i];

              //  printPaddedInt16b(pitch_a[i]);
              //  printPaddedInt16b(roll_a[i]);
              //  Serial.println("");

                  Serial.print(dt);
                  printPaddedInt16b(roll_g_lpf[i]);
                  printPaddedInt16b(pitch_g_lpf[i]);
                  printPaddedInt16b(yaw_g_lpf[i]);
                  Serial.println("");

                // Increment the count
                i++;

              }
            }
            cnt = i;

            Serial.print("IMU data collected, ");
            Serial.print(cnt);
            Serial.println(" samples");
            Serial.print("Sending ");

            time_start = millis();

            for (i=0; i<=cnt; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append(time_array[i]);
              tx_estring_value.append(",RC:");
              tx_estring_value.append(roll_g_lpf[i]);
              tx_estring_value.append(",PC:");
              tx_estring_value.append(pitch_g_lpf[i]);
              tx_estring_value.append(",YC:");
              tx_estring_value.append(yaw_g_lpf[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());

              if (i % 100 == 0) {
                Serial.print(".");
                }
              }
            
            Serial.println(" Done!");
            Serial.print("Total time: ");
            Serial.print((millis() - time_start)/1000);
            Serial.println(" seconds");

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
    }
}

void
setup()
{
    // Blick the LED 3 times to indicate we are up and running
    pinMode(LED_BUILTIN, OUTPUT);
    blink3();

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

    // Set up IMU
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    bool initialized = false;

    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
    // End IMU Setup

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

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}

void printPaddedInt16b( int16_t val ) {
  if (val > 0) {
    SERIAL_PORT.print(" ");
    if(val < 10000){ SERIAL_PORT.print("0"); }
    if(val < 1000 ){ SERIAL_PORT.print("0"); }
    if(val < 100  ){ SERIAL_PORT.print("0"); }
    if(val < 10   ){ SERIAL_PORT.print("0"); }
  } else {
    SERIAL_PORT.print("-");
    if(abs(val) < 10000){ SERIAL_PORT.print("0"); }
    if(abs(val) < 1000 ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 100  ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 10   ){ SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}