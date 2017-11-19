/*
 * Phat's Nguyen
 * HCMC University OF Technical And Education
 */
#include <Wire.h>
#include <I2Cdev.h>

#include "JJ_MPU6050_DMP_6Axis.h"
#include "define.h"
MPU6050 mpu;
void setup()
{
  Serial1_begin(115200);
  Serial1_println("mesg@Robot Configure");
  configure();
  InitServo();
  SWrite(1500);
}

void loop()
{
  timer_value = millis();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 18)
  {
    if (fifoCount > 18)
    {
      mpu.resetFIFO();
      return;
    }
    
    //Update time loop
    dt = (timer_value - timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    angle_adjusted = dmpGetPhi() + 1;//Set angle to natural balancing point

    mpu.resetFIFO();  //reset FIFO
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2) / 2;

    remoteSetting(setting);
    RobotControl();
    remoteServo(servo);

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0;
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;

    //Cập nhật giá trị PI điều khiển robot tiến tới
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle);

    //Cập nhật giá trị PD điều khiển robot cân bẳng
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    //giá trị đặt điều khiển robot xoay
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    //Tinh toan gia tri timer dieu khien dong co buoc
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    
    //Kiểm tra điều kiện khởi động động cơ điều khiển robot cân bằng
    if ((angle_adjusted < 20) && (angle_adjusted > -20) && robot_enable)
    {
      digitalWrite(4, LOW);  // Motors enable
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    }
    else
    {
      digitalWrite(4, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      steering = 0;
      throttle = 0;
    }
  } // End of new IMU data
}



