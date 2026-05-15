#include <ECE3.h>

uint16_t sensorValues[8];
int16_t weighted_sensorValues[8];

uint16_t minimum[8] = {690, 540, 591, 523, 454, 591, 570, 733};
uint16_t maximum[8] = {1741, 1904, 1859, 1375, 1320, 1859, 1881, 1717};
int16_t weights[8] = {8, 4, 2, 1, -1, -2, -4, -8};
int32_t error = 0; 


uint16_t sensorValues[8];

const int left_nslp_pin=31;
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;
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
  Serial.begin(9600);
  delay(2000);
}

void loop()
{
  ECE3_read_IR(sensorValues);
//Serial.println("raw readings");
  error = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    // Serial.print(sensorValues[i]);
    // Serial.print('\t');
    
  
    // weighted_sensorValues[i] =
    //   (10000*(constrain(sensorValues[i] - minimum[i],0, maximum[i]))/maximum[i]) * weights[i] / 4.0;
      error += (10000*(constrain(sensorValues[i] - minimum[i],0, maximum[i]))/maximum[i]) * weights[i] / 4.0;
  }
  //Serial.println("Weighted Sensor Values:");
  Serial.println("Error:");
 Serial.println(error);
// for (unsigned char i = 0; i < 8; i++)
//   {

//    //Serial.print(sensorValues[i] - minimum[i]);
//   //  Serial.print(1000*(constrain(sensorValues[i] - minimum[i],0, maximum[i]))/maximum[i]);
//   //   Serial.print('\t');
//   Serial.print(weighted_sensorValues[i]);
//  Serial.print('\t');
//   }
  
 //Serial.println();

  //Serial.print(error);
  //Serial.print('\t');
  //Serial.println();
  delay(1000);
}


float previous_error = 0;
float error_sum = 0;

float PID_algo(float Kp, float Ki, float Kd)
{
  float derivative = error - previous_error;
  error_sum += error;

  float output = Kp * error + Ki * error_sum + Kd * derivative;

  previous_error = error;

  return output;
}