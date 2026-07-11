// #include <ECE3.h>

// const int left_nslp_pin = 31;
// const int left_dir_pin  = 29;
// const int left_pwm_pin  = 40;

// const int right_nslp_pin = 11;
// const int right_dir_pin  = 30;
// const int right_pwm_pin  = 39;
// const float turn_time = 0.66; 

// // ---------------- MOTOR HELPER ----------------
// void setMotor(int pwm_pin, int dir_pin, int speed)
// {
//   speed = constrain(speed, -255, 255);

//   if (speed >= 0)
//   {
//     digitalWrite(dir_pin, LOW);   // forward
//     analogWrite(pwm_pin, speed);
//   }
//   else
//   {
//     digitalWrite(dir_pin, HIGH);  // reverse
//     analogWrite(pwm_pin, -speed);
//   }
// }

// void setup()
// {
//   pinMode(right_nslp_pin, OUTPUT);
//   pinMode(right_dir_pin, OUTPUT);
//   pinMode(right_pwm_pin, OUTPUT);

//   pinMode(left_nslp_pin, OUTPUT);
//   pinMode(left_dir_pin, OUTPUT);
//   pinMode(left_pwm_pin, OUTPUT);

//   // wake motor drivers
//   digitalWrite(right_nslp_pin, HIGH);
//   digitalWrite(left_nslp_pin, HIGH);

//   delay(500);

//   delay(2000); 
//   while time< turn time{ 

//   setMotor(left_pwm_pin, left_dir_pin, 150);    // forward max
//   setMotor(right_pwm_pin, right_dir_pin, -150); // reverse max
//   }
// }

// void loop()
// {
//   // opposite directions at max speed
//   s
// }

#include <ECE3.h>

const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

const float turn_time = 0.75; // seconds

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

void stopMotors()
{
  setMotor(left_pwm_pin, left_dir_pin, 0);
  setMotor(right_pwm_pin, right_dir_pin, 0);
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
  delay(2000);

  unsigned long startTime = millis();

  while (millis() - startTime < turn_time * 1000)
  {
    setMotor(left_pwm_pin, left_dir_pin, 75);    
    setMotor(right_pwm_pin, right_dir_pin, -75); 
  }

  stopMotors();
}

void loop()
{
  // Nothing here because the turn happens once in setup()
}