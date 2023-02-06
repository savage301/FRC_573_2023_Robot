// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Appendage.h"

/*
 * Remaps a number
 */
double Appendage::remapVal(double i, double threshold) {
  if (std::abs(i) > threshold) {
    i = i / std::abs(i) * threshold;
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

  m_frontRollerMotor = new rev::CANSparkMax{
      m_frontRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_backRollerMotor = new rev::CANSparkMax{
      m_backRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_armMotor =
      new rev::CANSparkMax{m_armId, rev::CANSparkMax::MotorType::kBrushless};
  m_shoulderMotor = new rev::CANSparkMax{
      m_shoulderId, rev::CANSparkMax::MotorType::kBrushless};
  arm_Encoder = new rev::SparkMaxRelativeEncoder{m_armMotor->GetEncoder(
      rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)};
  shoulder_Encoder = new frc::Encoder(2, 3, false);
  wrist_Encoder = new frc::Encoder(8, 9, false);

  lim_top = new frc::DigitalInput(6);
  lim_bot = new frc::DigitalInput(7);

  claw1_a_input = new frc::AnalogInput(0);
  edge1_a_input = new frc::AnalogInput(1);
  edge2_a_input = new frc::AnalogInput(2);

  p_backRollerCylinder =
      new frc::DoubleSolenoid(19, frc::PneumaticsModuleType::CTREPCM,
                              p_backRollerId_a, p_backRollerId_b);

  m_wristMotor = new rev::CANSparkMax(m_wristMotorId,
                                      rev::CANSparkMax::MotorType::kBrushless);
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

void Appendage::arm(double d) {
  double out = remapVal(d, .7);
  if ((lim_top->Get() && out > 0) || (lim_bot->Get() && out < 0))
    out = 0;
  m_armMotor->Set(out);
}

void Appendage::shoulder(double d) {
  d = remapVal(d, .7);
  m_shoulderMotor->Set(d);
}

bool Appendage::checkLim(double err, double lim) {
  if (std::abs(err) > lim)
    return false;
  else
    return true;
}

bool Appendage::shoulderPID(double tar) {
  double cur = shoulder_Encoder->GetDistance();
  double out = Shoulder_PIDController.Calculate(cur, tar);

  double softStopMinLim = 0;
  double softStopMaxLim = 1;
  if ((cur < softStopMinLim && out < 0) || (cur > softStopMaxLim && out > 0))
    out = 0;

  m_shoulderMotor->Set(out);
  if (checkLim(cur - out, 10))
    return true;
  return false;
}

bool Appendage::armPID(double tar) {
  double cur = arm_Encoder->GetPosition();
  double out = Arm_PIDController.Calculate(cur, tar);
  if ((lim_top->Get() && out > 0) || (lim_bot->Get() && out < 0))
    out = 0;
  m_armMotor->Set(out);

  if (checkLim(cur - out, 10))
    return true;
  return false;
}

double Appendage::calculateDistanceToLim() {
  double distanceToLim;
  double curPos = arm_Encoder->GetPosition();
  double curAng = shoulder_Encoder->GetDistance();
  int gearRatioArm = 1, gearRatioShoulder = 1;  // update to real
  // num * enc
  double armLength =
      gearRatioArm * curPos + 30;  // default unextended arm length
  double shoulderAng = gearRatioShoulder * curAng;
  distanceToLim = 78 - std::sin(shoulderAng) * armLength - 20.5;
  return distanceToLim;  // neg = exceed the limit
}

void Appendage::wrist(double d) {
  d = remapVal(d, .7);
  m_wristMotor->Set(d);
}

bool Appendage::wristPID(double tar) {
  double cur = wrist_Encoder->GetDistance();
  double out = Wrist_PIDController.Calculate(cur, tar);
  m_wristMotor->Set(out);

  if (checkLim(cur - out, 10))
    return true;
  return false;
}

#include <frc/smartdashboard/SmartDashboard.h>
#define pumpOut frc::SmartDashboard::PutNumber
void Appendage::pumpOutSensorVal() {
  double armCur = arm_Encoder->GetPosition();
  double wristCur = wrist_Encoder->GetDistance();
  double shoulderCur = shoulder_Encoder->GetDistance();
  pumpOut("Claw 1 AnalogInput", claw1_a_input->GetValue());
  pumpOut("edge 1 AnalogInput", edge1_a_input->GetValue());
  pumpOut("edge 2 AnalogInput", edge2_a_input->GetValue());
  pumpOut("Arm Encoder", armCur);
  pumpOut("Wrist Encoder", wristCur);
  pumpOut("Shoulder Encoder", shoulderCur);
}

bool Appendage::gamePieceInClaw() {
  int limUp = 500, limDown = 200;

  if (claw1_a_input->GetValue() > limDown && claw1_a_input->GetValue() < limUp)
    return true;

  return false;
}

void Appendage::appendageReset(bool isPneumaticsIn) {
  wrist(Appendage::wristOff);
  arm(Appendage::armOff);
  shoulder(Appendage::shoulderOff);

  frontRollerOff();
  backRollerOff();

  if (isPneumaticsIn)
    pneumaticsIn();  // let go
  else
    pneumaticsOut();
}

bool Appendage::checkEdge() {
  double lim = 500;
  if (edge1_a_input->GetValue() > lim || edge2_a_input->GetValue() > lim)
    return true;

  return false;
}
