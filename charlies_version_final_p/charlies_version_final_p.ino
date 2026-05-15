#include <ECE3.h>

uint16_t sensorValues[8];
int16_t weighted_sensorValues[8];

uint16_t minimum[8] = {690, 540, 591, 523, 454, 591, 570, 733};
uint16_t maximum[8] = {1741, 1904, 1859, 1375, 1320, 1859, 1881, 1717};
int16_t weights[8] = {8, 4, 2, 1, -1, -2, -4, -8};
int32_t error = 0; 

const int left_nslp_pin=31;
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;
float last_P = 0, last_I = 0, last_D = 0, last_output = 0;
void setup()
{
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

void loop()
{
  ECE3_read_IR(sensorValues);
//Serial.println("raw readings");
  error = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
      error += (10000*(constrain(sensorValues[i] - minimum[i],0, maximum[i]))/maximum[i]) * weights[i] / 4.0;
  }

  float output = PID_algo(0.005, 0, 0.0001);
  int mapped_output =  correction_to_motor_val(output);

// if (mapped_output == 0) {
//   drive(0, 80);
// }
//   if (mapped_output< 0){
//     drive(1, -1 * mapped_output);
//   }
//   if (mapped_output > 0){
//     drive(2, mapped_output);
//   }
int baseSpeed = 80;
int correction = correction_to_motor_val(output);

int leftSpeed = baseSpeed + correction;
int rightSpeed = baseSpeed - correction;

setMotor(left_pwm_pin, left_dir_pin, leftSpeed);
setMotor(right_pwm_pin, right_dir_pin, rightSpeed);
//   Serial.println("Error:");
//  Serial.println(error);
  

//   Serial.print("err:");   
//      Serial.print(error);
// Serial.print("\tP:");   
//    Serial.print(last_P, 2);
// Serial.print("\tI:");    
//   Serial.print(last_I, 2);
// Serial.print("\tD:");    
//   Serial.print(last_D, 2);
// Serial.print("\tout:");  
//   Serial.print(last_output, 2);
// Serial.print("\tcorr:");  
//  Serial.print(correction);
// Serial.print("\tL:");     
//  Serial.print(leftSpeed);
// Serial.print("\tR:");     
//  Serial.println(rightSpeed);
// delay(200);
}


float previous_error = 0;
float error_sum = 0;
unsigned long previous_time = 0;

// float PID_algo(float Kp, float Ki, float Kd)
// {
//   unsigned long current_time = millis();
//   float dt = (current_time - previous_time) / 1000.0;

//   if (dt <= 0) dt = 0.001;

//   float derivative = (error - previous_error) / dt;
//   error_sum += error * dt;

//   error_sum = constrain(error_sum, -10000, 10000);

//   float output = Kp * error + Ki * error_sum + Kd * derivative;

//   previous_error = error;
//   previous_time = current_time;

//   return output;
// }

// Add these globals near the top with the others

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


// direction:
// 0 = straight
// 1 = left
// 2 = right
// void drive(int direction, int speed)
// {
//   speed = constrain(speed, 0, 255);

//   switch(direction)
//   {
//     case 0: // straight
//       setMotor(left_pwm_pin, left_dir_pin, speed);
//       setMotor(right_pwm_pin, right_dir_pin, speed);
//       break;

//     case 1: // left turn
//       setMotor(left_pwm_pin, left_dir_pin, speed / 2);
//       setMotor(right_pwm_pin, right_dir_pin, speed);
//       break;

//     case 2: // right turn
//       setMotor(left_pwm_pin, left_dir_pin, speed);
//       setMotor(right_pwm_pin, right_dir_pin, speed / 2);
//       break;
//   }
// }

int correction_to_motor_val(float pid_output)
{
  pid_output = constrain(pid_output, -255, 255);
  return (int)pid_output;
}