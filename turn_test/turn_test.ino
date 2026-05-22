#include <ECE3.h>

const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ---------------- MOTOR HELPER ----------------
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

void setup()
{
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  // wake motor drivers
  digitalWrite(right_nslp_pin, HIGH);
  digitalWrite(left_nslp_pin, HIGH);

  delay(500);
}

void loop()
{
  // opposite directions at max speed
  setMotor(left_pwm_pin, left_dir_pin, 255);    // forward max
  setMotor(right_pwm_pin, right_dir_pin, -255); // reverse max
}