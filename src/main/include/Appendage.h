// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <numbers>
#include <frc2/command/PIDCommand.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogInput.h>
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
  void pneumaticsIn();
  void pneumaticsOut();

  // Telescoping Arm
  void arm(double d);
  bool armPID(double tar);

  // Shoulder
  void shoulder(double d);
  bool shoulderPID(double tar);

  // Arm + Shoulder
  double calculateDistanceToLim();

  // Wrist
  void wrist(double d);
  bool wristPID(double tar);

  // err = cur - tar
  bool checkLim(double err, double lim);

  void pumpOutSensorVal();

  // Claw UltSnd
  bool gamePieceInClaw();
  frc::AnalogInput* a_Input;

 private:
  // Claw Motors
  rev::CANSparkMax* m_backRollerMotor;
  rev::CANSparkMax* m_frontRollerMotor;

  // Telescoping Arm Motor
  rev::CANSparkMax* m_armMotor;
  rev::RelativeEncoder* arm_Encoder;
  frc::DigitalInput* lim_top;
  frc::DigitalInput* lim_bot;

  // Shoulder Motor
  rev::CANSparkMax* m_shoulderMotor;
  frc::Encoder* shoulder_Encoder;

  // Wrist Motor
  rev::CANSparkMax* m_wristMotor;
  frc::Encoder* wrist_Encoder;

  // Claw Rear Roller Cylinder
  frc::DoubleSolenoid* p_backRollerCylinder;

  double remapVal(double i, double threshold);

  frc2::PIDController Arm_PIDController{1.0, 0, 0};
  frc2::PIDController Shoulder_PIDController{1.0, 0, 0};
  frc2::PIDController Wrist_PIDController{1.0, 0, 0};

};