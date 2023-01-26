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
  m_frontRollerMotor =
      new rev::CANSparkMax{m_frontRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_backRollerMotor =
      new rev::CANSparkMax{m_backRollerId, rev::CANSparkMax::MotorType::kBrushless};
  m_armMotor =
      new rev::CANSparkMax{m_armId, rev::CANSparkMax::MotorType::kBrushless};
  m_shoulderMotor =
      new rev::CANSparkMax{m_shoulderId, rev::CANSparkMax::MotorType::kBrushless};

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

void Appendage::arm(double d){
  d=remapVal(d,.7);
  m_armMotor->Set(d);
}

void Appendage::shoulder(double d){
  d=remapVal(d,.7);
  m_shoulderMotor->Set(d);
}