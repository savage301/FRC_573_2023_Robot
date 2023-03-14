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

double Appendage::deadband(double i, double threshold) {
  if (std::abs(i) < threshold) {
    i = 0;
  }

  return i;
}

Appendage::Appendage() {
  m_frontRollerMotor = new rev::CANSparkMax{
      m_frontRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_backRollerMotor = new rev::CANSparkMax{
      m_backRollerId, rev::CANSparkMax::MotorType::kBrushless};

  m_armMotor =
      new rev::CANSparkMax{m_armId, rev::CANSparkMax::MotorType::kBrushless};
  arm_Encoder = new rev::SparkMaxRelativeEncoder{m_armMotor->GetEncoder(
      rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)};

  m_shoulderMotor = new rev::CANSparkMax{
      m_shoulderId, rev::CANSparkMax::MotorType::kBrushless};

  m_shoulderMotor->SetInverted(true);
  shoulder_Encoder = new frc::Encoder(6, 7, false);

  m_wristMotor = new rev::CANSparkMax(m_wristMotorId,
                                      rev::CANSparkMax::MotorType::kBrushless);

  m_wristMotor->SetInverted(true);
  wrist_Encoder = new frc::Encoder(8, 9, false);

  claw1_a_input = new frc::AnalogInput(0);
  claw2_a_input = new frc::AnalogInput(3);
  edge1_a_input = new frc::AnalogInput(1);
  edge2_a_input = new frc::AnalogInput(2);
#define pneumatics(a, b) \
  new frc::DoubleSolenoid(p_pcmId, frc::PneumaticsModuleType::CTREPCM, a, b)
  p_backRollerCylinder1 = pneumatics(p_Roller1Id_a, p_Roller1Id_b);
}

void Appendage::frontRollerIn() {
  m_frontRollerMotor->Set(0.50);
}

void Appendage::frontRollerOut() {
  m_frontRollerMotor->Set(-0.35);
}

void Appendage::frontRollerOff() {
  m_frontRollerMotor->Set(0);
}

void Appendage::backRollerIn() {
  m_backRollerMotor->Set(-0.50);
}

void Appendage::backRollerOut() {
  m_backRollerMotor->Set(0.35);
}

void Appendage::backRollerOff() {
  m_backRollerMotor->Set(0);
}

void Appendage::pneumaticsIn() {
  p_backRollerCylinder1->Set(frc::DoubleSolenoid::kForward);
}

void Appendage::pneumaticsOut() {
  p_backRollerCylinder1->Set(frc::DoubleSolenoid::kReverse);
}

void Appendage::arm(double d) {
  double out = remapVal(d, .7);
  out = deadband(out, 0.1);
  double cur = arm_Encoder->GetPosition();

  if (!unleashThePower) {
    if ((cur < arm_min && out < 0) || (cur > arm_max && out > 0))
      out = 0;
  }

  m_armMotor->Set(out);
}

void Appendage::shoulder(double d) {
  double out = remapVal(d, .7);
  out = deadband(out, 0.05);
  double cur = shoulder_Encoder->GetDistance();

  if (!unleashThePower) {
    if ((cur < shoulder_min && out > 0) || (cur > shoulder_max && out < 0))
      out = 0;
  }

  m_shoulderMotor->Set(out);
}

bool Appendage::checkLim(double err, double lim) {
  if (std::abs(err) > lim)
    return false;
  else
    return true;
}

bool Appendage::shoulderPID(double tar) {
  double p = -.005, i = 0, d = 0;
  double limit = 10, maxval = .7;
  double outlimit = 50;
  Shoulder_PIDController.SetPID(p, i, d);
  double cur = shoulder_Encoder->GetDistance();
  double out = Shoulder_PIDController.Calculate(cur, tar);

  if ((cur < shoulder_min && out > 0) || (cur > shoulder_max && out < 0))
    out = 0;

  if (checkLim(cur - tar, limit)) {
    m_shoulderMotor->Set(0);
  } else {
    out = remapVal(out, maxval);
    m_shoulderMotor->Set(out);
  }

  if (checkLim(cur - tar, outlimit))
    return true;
  else
    return false;
}

bool Appendage::armPID(double tar) {
  double p = .1, i = 0, d = 0;
  double limit = 5, maxval = 1;
  double outlimit = 10;
  Arm_PIDController.SetPID(p, i, d);
  double cur = arm_Encoder->GetPosition();
  double out = Arm_PIDController.Calculate(cur, tar);

  if ((cur < arm_min && out < 0) || (cur > arm_max && out > 0))
    out = 0;

  if (checkLim(cur - tar, limit)) {
    m_armMotor->Set(0);
  } else {
    out = remapVal(out, maxval);
    m_armMotor->Set(out);
  }

  if (checkLim(cur - tar, outlimit))
    return true;
  else
    return false;
}

double Appendage::calculateDistanceToLim() {
  double distanceToLim;
  double curPos = arm_Encoder->GetPosition();
  double curAng = shoulder_Encoder->GetDistance();
  double curWAng = wrist_Encoder->GetDistance();
  int gearRatioArm = 1, gearRatioShoulder = 1,
      gearRatioWrist = 1;  // update to real
  // num * enc
  double armLength =
      gearRatioArm * curPos + 24;  // default unextended arm length
  double shoulderAng = gearRatioShoulder * curAng;
  double wristLength = 20, wristAngle = gearRatioWrist * curWAng;
  distanceToLim = 78 - std::sin(shoulderAng) * armLength - 20.75 -
                  wristLength * std::sin(wristAngle);
  return distanceToLim;  // neg = exceed the limit
}

void Appendage::wrist(double d) {
  double out = remapVal(d, .7);
  out = deadband(out, 0.2);
  double cur = wrist_Encoder->GetDistance();

  if (!unleashThePower) {
    if ((cur < wrist_min && out > 0) || (cur > wrist_max && out < 0))
      out = 0;
  }

  m_wristMotor->Set(out);
}

bool Appendage::wristPID(double tar) {
  double p = -.01, i = 0, d = 0;
  double limit = 50, maxval = 1;
  double outlimit = 75;
  Wrist_PIDController.SetPID(p, i, d);
  double cur = wrist_Encoder->GetDistance();
  double out = Wrist_PIDController.Calculate(cur, tar);

  if ((cur < wrist_min && out > 0) || (cur > wrist_max && out < 0))
    out = 0;

  if (checkLim(cur - tar, limit)) {
    m_wristMotor->Set(0);
  } else {
    out = remapVal(out, maxval);

    m_wristMotor->Set(out);
  }
  if (checkLim(cur - tar, outlimit))
    return true;
  else
    return false;
}

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

bool Appendage::isGamePieceInClaw() {
  int limUp = 500, limDown = 200;

  if ((claw1_a_input->GetValue() > limDown &&
       claw1_a_input->GetValue() < limUp) ||
      (claw2_a_input->GetValue() > limDown &&
       claw2_a_input->GetValue() < limUp))
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

/*
 * checks if a sensor is working
 * @param canMotor - name of the can motor
 *        canEncoder - name of the can encoder
 *        aInput - name of the analog input
 *        last - last value to check
 * @return true - if last is NOT equal to current
 *         false - if last is equal to current
 */
bool Appendage::isSensorWorking(rev::CANSparkMax* canMotor,
                                rev::RelativeEncoder* canEncoder, double last) {
  double cur = 0;
  bool ret = false;
  if (std::abs(canMotor->GetOutputCurrent()) > 1) {
    cur = canEncoder->GetPosition();
    if (cur != last) {
      ret = true;
    }
  }
  last = cur;
  return ret;
}

bool Appendage::isSensorWorking(rev::CANSparkMax* canMotor,
                                frc::Encoder* frcEncoder, double last) {
  double cur = 0;
  bool ret = false;
  if (std::abs(canMotor->GetOutputCurrent()) > 1) {
    cur = frcEncoder->GetDistance();
    if (cur != last) {
      ret = true;
    }
  }
  last = cur;
  return ret;
}

bool Appendage::isSensorWorking(frc::AnalogInput* aInput, double last) {
  double cur = 0;
  bool ret = false;
  cur = aInput->GetValue();
  if (cur != last) {
    ret = true;
  }
  last = cur;
  return ret;
}

bool Appendage::getArmWorking() {
  return isSensorWorking(m_armMotor, arm_Encoder, lastArm);
}

bool Appendage::getShoulderWorking() {
  return isSensorWorking(m_shoulderMotor, shoulder_Encoder, lastShoulder);
}

bool Appendage::getWristWorking() {
  return isSensorWorking(m_wristMotor, wrist_Encoder, lastWrist);
}

bool Appendage::getAnalogWorking() {
  bool claw1Working = false, claw2Working = false, ret;
  claw1Working = isSensorWorking(claw1_a_input, lastClaw1);
  claw2Working = isSensorWorking(claw2_a_input, lastClaw2);
  if (claw1Working == true && claw2Working == true)
    ret = true;
  if (claw1Working == false || claw2Working == false)
    ret = false;
  return ret;
}

double Appendage::analogToDistance(double i) {
  // from https://www.openhacks.com/uploadsproductos/wiki_4j.pdf
  return 2076 / (i - 11);
}

double Appendage::getClaw1() {
  return analogToDistance(claw1_a_input->GetValue());
}

double Appendage::getClaw2() {
  return analogToDistance(claw2_a_input->GetValue());
}

bool Appendage::getArmExtended() {
  return arm_Encoder->GetPosition() > 1000;  // update threshold
}
