// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <numbers>

class Appendage {
 public:
  Appendage();

  // Claw - front
  void frontRollerIn();
  void frontRollerOut();
  void frontRollerOff();
  // Claw - back
  void backRollerIn();
  void backRollerOut();
  void backRollerOff();

  // Telescoping Arm
  void arm(double d);

  // Shoulder
  void shoulder(double d);

 private:
  // Claw Motors
  rev::CANSparkMax* m_backRollerMotor;
  rev::CANSparkMax* m_frontRollerMotor;

  // Telescoping Arm Motor
  rev::CANSparkMax* m_armMotor;

  // Shoulder Motor
  rev::CANSparkMax* m_shoulderMotor;

  double remapVal(double i, double threshold);

};