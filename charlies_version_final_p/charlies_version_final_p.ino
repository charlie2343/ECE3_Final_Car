  #include <ECE3.h>

  uint16_t sensorValues[8];
  int16_t weighted_sensorValues[8];
  const int MAX_SENSOR_THRESHOLD = 9500;
  int maxReadCount = 0;
  float turnScore = 0;
  unsigned long lastTurnTime = 0;

  const float TURN_DECAY = 0.90;        // lower = forget faster
  const float TURN_THRESHOLD = 1.8;     // tune this
  const float P_DEADBAND = 1.0;         // ignore tiny steering corrections

  uint16_t minimum[8] = {690, 540, 591, 523, 454, 591, 570, 733};
  uint16_t maximum[8] = {1741, 1904, 1859, 1375, 1320, 1859, 1881, 1717};
  int16_t weights[8] = {8, 4, 2, 1, -1, -2, -4, -8};
  int32_t error = 0; 

  const int left_nslp_pin=31;
  const int left_dir_pin=29;
  const int left_pwm_pin=40;
  int baseSpeed = 220;
  const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
  const int right_dir_pin=30;
  const int right_pwm_pin=39;
  const int LED_RF = 41;
  float last_P = 0, last_I = 0, last_D = 0, last_output = 0;

  // Bump switches (active-low, need INPUT_PULLUP)
  const int bump0_pin = 25;  // P4.0
  const int bump1_pin = 27;  // P4.2
  const int bump2_pin = 6;   // P4.3
  const int bump3_pin = 28;  // P4.5
  const int bump4_pin = 8;   // P4.6
  const int bump5_pin = 29;  // P4.7

  bool end_reached = false;
  bool turn_reached = false; 

  const int bump_pins[6] = {bump0_pin, bump1_pin, bump2_pin,
                            bump3_pin, bump4_pin, bump5_pin};

                            // Tunable parameters (modified by bumpers)
  float Kp = 0.007;
  float Ki = 0.001;
  float Kd = 0.0005;

  // Bumper debounce state
  bool bump_prev[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
  unsigned long bump_last_time[6] = {0, 0, 0, 0, 0, 0};
  const unsigned long BUMP_DEBOUNCE_MS = 200;
  float previous_error = 0;
float error_sum = 0;
unsigned long previous_time = 0;

  void setup()
  {
    
    for (int i = 0; i < 6; i++) {
    pinMode(bump_pins[i], INPUT_PULLUP);
  }

    pinMode(right_nslp_pin,OUTPUT);
    pinMode(right_dir_pin,OUTPUT);
    pinMode(right_pwm_pin,OUTPUT);

    digitalWrite(right_dir_pin,LOW);
    digitalWrite(right_nslp_pin,HIGH);

    pinMode(left_nslp_pin,OUTPUT);
    pinMode(left_dir_pin,OUTPUT);
    pinMode(left_pwm_pin,OUTPUT);

    digitalWrite(left_dir_pin,LOW);
    digitalWrite(left_nslp_pin,HIGH);

    pinMode(LED_RF, OUTPUT);

    ECE3_Init();
    Serial.begin(115200);
    delay(2000);
  }

  unsigned long detection_time = 0;

  void loop()
  {
    ECE3_read_IR(sensorValues);

    if (isPerpendicularLine() && (detection_time == 0 || (millis() - detection_time) > 2000))
{
  maxReadCount++;
}
    else
    {
      maxReadCount = 0;
    }

    if (maxReadCount >= 2 && !turn_reached)
    {
      turnAtMaxSpeed();
      maxReadCount = 0;
      error_sum = 0;
      previous_error = 0;
      detection_time = millis();
      turn_reached = true; 
      return;
    }

    else if (maxReadCount >= 2 && turn_reached)
    {
      end_reached = true;
    }



    error = 0;
    for (unsigned char i = 0; i < 8; i++)
    {
      int normalized = 10000 *
        constrain(sensorValues[i] - minimum[i], 0, maximum[i] - minimum[i])
        / (maximum[i]);

      error += normalized * weights[i] / 4.0;
    }

    float output = PID_algo(Kp, Ki, Kd);
    int correction = correction_to_motor_val(output);

    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    handle_bumpers();

    if(!end_reached){
    setMotor(left_pwm_pin, left_dir_pin, leftSpeed);
    setMotor(right_pwm_pin, right_dir_pin, rightSpeed);
    } 
    else{ 
    setMotor(left_pwm_pin, left_dir_pin, 0);
    setMotor(right_pwm_pin, right_dir_pin, 0);
    }

    if(millis() - detection_time < 2000){ 
  Kp = 0.005;
  Ki = 0.001;
  Kd = 0.0007;

    }
  }



  float PID_algo(float Kp, float Ki, float Kd)
  {
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    if (dt <= 0) dt = 0.001;

    float derivative = (error - previous_error) / dt;
    error_sum += error * dt;
    error_sum = constrain(error_sum, -10000, 10000);

    float P = Kp * error;
    float I = Ki * error_sum;
    float D = Kd * derivative;
    float output = P + I + D;

    last_P = P;
    last_I = I;
    last_D = D;
    last_output = output;

    previous_error = error;
    previous_time = current_time;

    return output;
  }


  // ---------------- MOTOR HELPERS ----------------
  void setMotor(int pwm_pin, int dir_pin, int speed)
  {
    speed = constrain(speed, -255, 255);

    if (speed >= 0)
    {
      digitalWrite(dir_pin, LOW);   // forward
      analogWrite(pwm_pin, speed);
    }
    else
    {
      digitalWrite(dir_pin, HIGH);  // reverse
      analogWrite(pwm_pin, -speed);
    }
  }

  int correction_to_motor_val(float pid_output)
  {
    pid_output = constrain(pid_output, -255, 255);
    return (int)pid_output;
  }


  void handle_bumpers() {
    unsigned long now = millis();
    for (int i = 0; i < 6; i++) {
      bool state = digitalRead(bump_pins[i]);

      // Detect falling edge (HIGH -> LOW = press), with debounce
      if (state == LOW && bump_prev[i] == HIGH &&
          (now - bump_last_time[i]) > BUMP_DEBOUNCE_MS)
      {
        bump_last_time[i] = now;

        switch (i) {
          case 0:  // Bump 0: Kp += 0.01
            Kp += 0.001;
            Serial.print(">>> Kp = "); Serial.println(Kp, 4);
            break;
          case 2:  // Bump 1: Ki += 0.001
            Ki += 0.001;
            Serial.print(">>> Ki = "); Serial.println(Ki, 4);
            break;
          case 1:  // Bump 2: Kd += 0.01
            Kd += 0.0001;
            Serial.print(">>> Kd = "); Serial.println(Kd, 4);
            break;
          case 3:  // Bump 3: baseSpeed += 20
            baseSpeed += 20;
            if (baseSpeed > 255) baseSpeed = 255;
            Serial.print(">>> baseSpeed = "); Serial.println(baseSpeed);
            break;
          // case 4, 5: unused (reserve for "decrease" buttons if useful)
        }
      }
      bump_prev[i] = state;
    }
  }

  bool isPerpendicularLine()
  {
    for (int i = 0; i < 8; i++)
    {
      int normalized = 10000 * constrain(sensorValues[i] - minimum[i], 0, maximum[i]) / maximum[i];

      if (normalized < MAX_SENSOR_THRESHOLD)
      {
        return false;
      }
    }

    return true;
  }

  void turnAtMaxSpeed()
  {
    setMotor(left_pwm_pin, left_dir_pin, 75);
    setMotor(right_pwm_pin, right_dir_pin, -75);
    delay(780);
    // setMotor(left_pwm_pin, left_dir_pin, 0);
    // setMotor(right_pwm_pin, right_dir_pin, 0);
    // delay(50);
  }