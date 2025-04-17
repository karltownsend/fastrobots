#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <math.h>
#include "constants.h"
#include "utility.h"
#include "car.h"
#include "imu.h"
#include "tof.h"
#include "pid.h"

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
unsigned long time_start, time_now, time_stamp, time_prev;
int cnt, period;
float dt, alpha, alpha_lpf;

#define data_size 3000
int max_time = 8000;
int time_array[data_size];
float temp_array[data_size];
float pitch_a[data_size], roll_a[data_size];
float pitch_gyro, roll_gyro, yaw_gyro;
float roll_comp = 0.0, pitch_comp = 0.0, yaw_comp = 0.0;
float pitch_a_lpf[data_size], roll_a_lpf[data_size];
float pitch_a_lpfi, pitch_a_lpfi1, roll_a_lpfi, roll_a_lpfi1;
float pitch_g[data_size], roll_g[data_size], yaw_g[data_size];
float pitch_g_lpf[data_size], roll_g_lpf[data_size], yaw_g_lpf[data_size];
float fdist[data_size], sdist[data_size], pid[data_size], motor_pwm[data_size];

float front_dist, current_angle;
float kp = 0.015, ki = 0.001, kd = 0.0, pid_out = 100;
float kp_r, ki_r, kd_r, pid_out_angle;
float wall_dist = 305;  // 1 foot = 305 mm
float set_angle;

float correction = 1.2;  // increase left wheel speed to keep the car going straight
int pwm_max = MAX, deadzone = 40;
float P[data_size], I[data_size], D[data_size];

enum CommandTypes {
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
  GET_IMU_DATA,
  GET_RANGE,
  SET_PWM,
  GO_WALL,
  GET_WALL_DATA,
  SET_PID_MAX
};

// Create the car and sensor objects
Car car;
Imu imu;
Tof sideTOF;
Tof frontTOF(FRONT_TOF_SHUTDOWN);
PID LinearPID(&front_dist, &pid_out, &wall_dist, kp, ki, kd);
PID RotatePID(&current_angle, &pid_out_angle, &set_angle, kp_r, ki_r, kd_r);

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

      // Extract the next values from the command string as an integer
      if (!robot_cmd.get_next_value(int_a))
        return;

      if (!robot_cmd.get_next_value(int_b))
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

      float float_a, float_b, float_c;

      // Extract the next values from the command string as a float
      if (!robot_cmd.get_next_value(float_a))
        return;

      if (!robot_cmd.get_next_value(float_b))
        return;

      if (!robot_cmd.get_next_value(float_c))
        return;

      Serial.print("Three Floats: ");
      Serial.print(float_a, 4);
      Serial.print(", ");
      Serial.print(float_b, 4);
      Serial.print(", ");
      Serial.println(float_c, 4);
      break;

    /*
      Add a prefix and postfix to the string value extracted from the command string
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
      break;

    case GET_TIME_MILLIS_LOOP:

      time_start = millis();
      cnt = 0;
      while (millis() - time_start < 2000) {
        tx_estring_value.clear();
        tx_estring_value.append("T: " + millis());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        cnt++;
      }

      Serial.print("Total count: ");
      Serial.println(cnt);
      break;

    case STORE_TIME_DATA:
      for (i = 0; i < 500; i++) {
        time_array[i] = millis();
        temp_array[i] = getTempDegC();
      }
      Serial.println("Stored 500 time samples");
      break;

    case SEND_TIME_DATA:
      for (i = 0; i < 500; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append(time_array[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
      }
      Serial.println("Sent 500 time samples");
      break;

    case GET_TEMP_READINGS:
      for (i = 0; i < 500; i++) {
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

      Serial.println("Starting IMU collection");

      time_start = millis();
      time_array[0] = time_start;
      roll_a[0] = 0;
      pitch_a[0] = 0;
      roll_a_lpf[0] = 0;
      pitch_a_lpf[0] = 0;
      roll_g[0] = 0;
      pitch_g[0] = 0;
      yaw_g[0] = 0;
      roll_g_lpf[0] = 0;
      pitch_g_lpf[0] = 0;
      yaw_g_lpf[0] = 0;

      i = 1;

      while (millis() - time_start < 5000) {
        if (imu.dataReady()) {
          time_array[i] = millis();

          imu.getAGMT();  // The values are only updated when you call 'getAGMT'

          // Get the IMU accelerometer data
          //
          // Pitch (theta) = atan2(ax, az)
          // Roll (phi)    = atan2(ay, az)

          pitch_a[i] = atan2(imu.accX(), imu.accZ()) * 180 / M_PI;
          roll_a[i] = atan2(imu.accY(), imu.accZ()) * 180 / M_PI;

          alpha_lpf = 0.1;  // class suggets alpha = 0.02
          pitch_a_lpf[i] = alpha_lpf * pitch_a[i] + (1 - alpha_lpf) * pitch_a_lpf[i - 1];
          roll_a_lpf[i] = alpha_lpf * roll_a[i] + (1 - alpha_lpf) * roll_a_lpf[i - 1];
          pitch_a_lpf[i - 1] = pitch_a_lpf[i];
          roll_a_lpf[i - 1] = roll_a_lpf[i];

          // Make sure dt is in seconds!
          dt = (time_array[i] - time_array[i - 1]) / 1000.0;
          alpha = 0.01;

          // Get the IMU gyro data
          roll_g[i] = roll_g[i - 1] + imu.gyrX() * dt;
          pitch_g[i] = pitch_g[i - 1] + imu.gyrY() * dt;
          yaw_g[i] = yaw_g[i - 1] + imu.gyrZ() * dt;

          roll_g_lpf[i] = (1 - alpha) * (roll_g_lpf[i - 1] + roll_g[i]) + alpha * roll_a_lpf[i];
          pitch_g_lpf[i] = (1 - alpha) * (pitch_g_lpf[i - 1] + pitch_g[i]) + alpha * pitch_a_lpf[i];

          roll_g_lpf[i] = (1 - alpha) * (roll_g_lpf[i - 1] + imu.gyrX() * dt) + alpha * roll_a_lpf[i];
          pitch_g_lpf[i] = (1 - alpha) * (pitch_g_lpf[i - 1] + imu.gyrY() * dt) + alpha * pitch_a_lpf[i];
          //yaw_g_lpf[i]   = yaw_g_lpf[i]   + yaw_g[i];
          yaw_g_lpf[i] = yaw_g_lpf[i] + imu.gyrZ() * dt;

          // Increment the count
          i++;
        }
      }
      cnt = i;

      Serial.print("IMU data collected, ");
      Serial.print(cnt);
      Serial.print(" samples in 5 sec, ");
      Serial.print(5000.0 / cnt);
      Serial.println(" ms each.");
      Serial.print("Sending ");

      time_start = millis();

      for (i = 0; i <= cnt; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:" + time_array[i]);

        tx_estring_value.append(",RA:");
        tx_estring_value.append(roll_a[i]);
        tx_estring_value.append(",PA:");
        tx_estring_value.append(pitch_a[i]);

        tx_estring_value.append(",RL:");
        tx_estring_value.append(roll_a_lpf[i]);
        tx_estring_value.append(",PL:");
        tx_estring_value.append(pitch_a_lpf[i]);

        tx_estring_value.append(",RG:");
        tx_estring_value.append(roll_g[i]);
        tx_estring_value.append(",PG:");
        tx_estring_value.append(pitch_g[i]);
        tx_estring_value.append(",YG:");
        tx_estring_value.append(yaw_g[i]);

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
      Serial.print((millis() - time_start) / 1000.0);
      Serial.println(" seconds");
      break;

    case GET_RANGE:

      // Extract the pwm value from the command string as an integer
      if (!robot_cmd.get_next_value(max_time))
        return;

      Serial.println("Get Range ...");

      time_start = millis();
      frontTOF.startRanging();

      // Loop until we run out of time
      while (time_now < (time_start + max_time)) {

        time_now = millis();
        front_dist = frontTOF.getRangeExtrapolation();

        Serial.print(millis());
        Serial.print("   ");
        Serial.println(front_dist);
      }

      break;

    case SET_PWM:

      float pwm;
      int pwm_delay;

      // Extract the pwm value from the command string as an integer
      if (!robot_cmd.get_next_value(pwm))
        return;

      // Extract the movement time from the command string as an integer
      if (!robot_cmd.get_next_value(pwm_delay))
        return;
            
      car.setLinearSpeed(pwm);
      delay(pwm_delay);
      car.stop();

      break;

    case GO_WALL:

      // Init variables
      float pwm_value;
      LinearPID.begin();

      // Extract the optional distance to the wall from the command string as a float
      //if (robot_cmd.get_next_value(wall_dist_f))
      //  wall_dist = (double)wall_dist_f;

      time_start = millis();

      // Return the variables for this routine
      tx_estring_value.clear();
      tx_estring_value.append("KP:");
      tx_estring_value.append(kp);
      tx_estring_value.append(" KI:");
      tx_estring_value.append(ki);
      tx_estring_value.append(" KD:");
      tx_estring_value.append(kd);
      tx_estring_value.append(" PWM_MAX:");
      tx_estring_value.append(pwm_max);
      tx_estring_value.append(" DEADZONE:");
      tx_estring_value.append(deadzone);
      tx_estring_value.append(" CORRECTION:");
      tx_estring_value.append(correction);
      tx_estring_value.append(" WALL_DIST:");
      tx_estring_value.append(wall_dist);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      // start ranging - once started it continues until stop is called
      frontTOF.startRanging();

      // Start the car
      pid_out = 100;
      pwm_value = car.setLinearSpeed(pid_out);

      time_now = millis();
      i = 0;

      // Loop until we get less than 5 mm or run out of time
      while (!(fabs(front_dist - wall_dist) < 5.0 && fabs(pwm_value) < 1.0)  && time_now < (time_start + max_time)) {

        front_dist = max(frontTOF.getRangeExtrapolation(), 20.0);  // accounts for the setback between the front TOF and the wheels
        LinearPID.compute();  // PID controller automatically receives front_dist and modifies pid_out
        pwm_value = car.setLinearSpeed(pid_out);

        // store data into arrays
        LinearPID.getPID(&P[i], &I[i], &D[i]);
        time_array[i] = time_now;
        fdist[i] = front_dist;
        pid[i] = pid_out;
        motor_pwm[i] = pwm_value;

        // update loop variables
        i++;
        time_now = millis();
      }

      //end loop, stop motors and ToF sensor
      cnt = i;
      car.stop();
      frontTOF.stop();

      tx_estring_value.clear();
      tx_estring_value.append("  Finished Go Wall ... ");
      tx_estring_value.append(cnt);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      break;

    case GET_WALL_DATA:

      Serial.println("Sending wall data");
      tx_estring_value.clear();
      tx_estring_value.append("  Sending PID data ...");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      time_start = millis();

      for (i = 0; i <= cnt; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append(time_array[i]);

        tx_estring_value.append(",RA:");
        tx_estring_value.append(fdist[i]);
        tx_estring_value.append(",PD:");
        tx_estring_value.append(pid[i]);
        tx_estring_value.append(",PW:");
        tx_estring_value.append(motor_pwm[i]);
        tx_estring_value.append(",KP:");
        tx_estring_value.append(P[i]);
        tx_estring_value.append(",KI:");
        tx_estring_value.append(I[i]);
        tx_estring_value.append(",KD:");
        tx_estring_value.append(D[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        if (i % 100 == 0)
          Serial.print(".");
      }

      Serial.println(" Done!");
      Serial.print("Total time: ");
      Serial.print((millis() - time_start) / 1000.0);
      Serial.println(" seconds");

      tx_estring_value.clear();
      tx_estring_value.append("  Data sent!");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      break;

    case SET_PID_MAX:

      // use floats to get the values over BLE, then convert to doubles for the PID controller
      float correction;

      // Extract the PID value from the command string as floats
      if (!robot_cmd.get_next_value(kp))
        return;

      if (!robot_cmd.get_next_value(ki))
        return;

      if (!robot_cmd.get_next_value(kd))
        return;

      // Extract the pwm_max from the command string as an integer
      if (!robot_cmd.get_next_value(pwm_max))
        return;

      // Extract the deadzone from the command string as an integer
      if (!robot_cmd.get_next_value(deadzone))
        return;

      // Extract the wall distance from the command string as an integer
      if (!robot_cmd.get_next_value(wall_dist))
        return;

      // Extract the c orrection from the command string as an integer
      if (!robot_cmd.get_next_value(correction))
        return;

      LinearPID.setConstants(kp, ki, kd);
      car.setDeadzone((byte)deadzone);
      car.setPwmMax(pwm_max);
      car.setCorrection(correction);

      Serial.print("Kp: ");
      Serial.print(kp, 4);
      Serial.print("  Ki: ");
      Serial.print(ki, 4);
      Serial.print("  Kd: ");
      Serial.print(kd, 4);
      Serial.print("  PWM_MAX: ");
      Serial.print(pwm_max);
      Serial.print("  DEADZONE: ");
      Serial.println(deadzone);

      tx_estring_value.clear();
      tx_estring_value.append("KP:");
      tx_estring_value.append(kp);
      tx_estring_value.append(" KI:");
      tx_estring_value.append(ki);
      tx_estring_value.append(" KD:");
      tx_estring_value.append(kd);
      tx_estring_value.append(" PWM_MAX:");
      tx_estring_value.append(pwm_max);
      tx_estring_value.append(" DEADZONE:");
      tx_estring_value.append(deadzone);
      tx_estring_value.append(" WALL_DIST:");
      tx_estring_value.append(wall_dist);
      tx_estring_value.append(" CORRECTION:");
      tx_estring_value.append(correction);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

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

void setup() {
  // Setup serial monitor
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

  // Set up IMU
  Serial.println("\n\nInitializing IMU");
  imu.begin();

  // Set up motor drivers
  Serial.println("Intializing motor drivers");
  car.begin();
  car.setDeadzone(deadzone);
  car.setCorrection(1.1);

  // Set up TOF sensors
  // Change the sideTOF address
  frontTOF.sensorOff();
  sideTOF.setI2CAddress(SIDE_TOF_ADDRESS);
  frontTOF.sensorOn();

  if (sideTOF.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Side Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (frontTOF.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Front Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  // Set distance mode to Long for both TOF sensors
  frontTOF.setDistanceModeLong();
  sideTOF.setDistanceModeLong();

  Serial.println("ToF Sensors online!");

  // Output MAC Address and start BLE advertising
  Serial.println("Advertising BLE with MAC: " + BLE.address());
  BLE.advertise();

  // Blink the built-in LED 3 times to indicate we are up and running
  blink(3);
}

void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {

    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000)
      tx_float_value = 0;
    previousMillis = currentMillis;
  }
}

void read_data() {
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written())
    handle_command();
}

void loop() {

  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.println("Connected to: " + central.address());

    // While central is connected
    while (central.connected()) {
      // Send data
      write_data();

      // Read data
      read_data();
    }
    car.stop();
    Serial.println("Disconnected");
  }
}