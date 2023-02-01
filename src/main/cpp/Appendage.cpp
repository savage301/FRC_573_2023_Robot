// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "Appendage.h"


/*
 * Remaps a number
 */
double Appendage::remapVal(double i, double threshold)
{
    if (abs(i) > threshold)
    {
        i = i/abs(i) * threshold;
    }

    return i;
}

Appendage::Appendage() {
  int m_frontRollerId = 14;
  int m_backRollerId = 15;
  int m_armId = 16;
  int m_shoulderId = 17;
  int p_backRollerId_a = 0;
  int p_backRollerId_b = 1;
  int m_wristMotorId = 18;


  testMotor =
      new rev::CANSparkMax{1, rev::CANSparkMax::MotorType::kBrushless};
  testEncoder = 
      new rev::SparkMaxRelativeEncoder{testMotor->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};

  m_frontRollerMotor =
      new rev::CANSparkMax{m_frontRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_backRollerMotor =
      new rev::CANSparkMax{m_backRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_armMotor =
      new rev::CANSparkMax{m_armId, rev::CANSparkMax::MotorType::kBrushless};
  m_shoulderMotor =
      new rev::CANSparkMax{m_shoulderId, rev::CANSparkMax::MotorType::kBrushless};
  arm_Encoder = 
      new rev::SparkMaxRelativeEncoder{m_armMotor->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
  shoulder_Encoder = new frc::Encoder(2, 3, false);

  lim_top = new frc::DigitalInput(6);
  lim_bot = new frc::DigitalInput(7);

  p_backRollerCylinder = new frc::DoubleSolenoid(frc::PneumaticsModuleType::CTREPCM, p_backRollerId_a, p_backRollerId_b);

  m_wristMotor = new rev::CANSparkMax(m_wristMotorId, rev::CANSparkMax::MotorType::kBrushless);
}

void Appendage::frontRollerIn() {
  m_frontRollerMotor->Set(1);
}

void Appendage::frontRollerOut() {
  m_frontRollerMotor->Set(-1);
}

void Appendage::frontRollerOff() {
  m_frontRollerMotor->Set(0);
}

void Appendage::backRollerIn() {
  m_backRollerMotor->Set(1);
}

void Appendage::backRollerOut() {
  m_backRollerMotor->Set(-1);
}

void Appendage::backRollerOff() {
  m_backRollerMotor->Set(0);
}

void Appendage::pneumaticsIn() {
  p_backRollerCylinder->Set(frc::DoubleSolenoid::kForward);
}

void Appendage::pneumaticsOut() {
  p_backRollerCylinder->Set(frc::DoubleSolenoid::kReverse);
}

void Appendage::arm(double d){
  double out=remapVal(d,.7);
  if ((lim_top->Get() && out > 0) || (lim_bot->Get() && out < 0))
    out = 0;
  m_armMotor->Set(out);
}

void Appendage::shoulder(double d){
  d=remapVal(d,.7);
  m_shoulderMotor->Set(d);
}

bool Appendage::shoulderPID(double tar) {
  double cur = shoulder_Encoder->GetDistance();
  double out = Shoulder_PIDController.Calculate(cur, tar);

  double softStopMinLim = 0;
  double softStopMaxLim = 1;
  if ((cur < softStopMinLim && out < 0) || (cur > softStopMaxLim && out > 0))
    out = 0;

  m_shoulderMotor->Set(out);
  if  (cur == out)
      return true;
  return false;
}

bool Appendage::armPID(double tar) {
  double cur = arm_Encoder->GetPosition();
  double out = Arm_PIDController.Calculate(cur, tar);
  if ((lim_top->Get() && out > 0) || (lim_bot->Get() && out < 0))
    out = 0;
  m_armMotor->Set(out);

  if (cur == out)
    return true;
  return false;
}

double Appendage::calculateDistanceToLim() {
  double distanceToLim;
  double curPos = arm_Encoder->GetPosition();
  double curAng = shoulder_Encoder->GetDistance();
  int gearRatioArm = 1, gearRatioShoulder = 1; // update to real
  // num * enc
  double armLength = gearRatioArm * curPos + 30; // default unextended arm length
  double shoulderAng = gearRatioShoulder * curAng;
  distanceToLim = 78 - sin(shoulderAng) * armLength - 20.5;
  return distanceToLim; // neg = exceed the limit
}

bool Appendage::wristPID(double tar) {
  double cur = wrist_Encoder->GetDistance();
  double out = Wrist_PIDController.Calculate(cur, tar);
  m_wristMotor->Set(out);
  if (cur == out)
      return true;
  return false;
}
#include <frc/smartdashboard/SmartDashboard.h>

void Appendage::print() {
  frc::SmartDashboard::PutNumber("test",testEncoder->GetPosition());
}