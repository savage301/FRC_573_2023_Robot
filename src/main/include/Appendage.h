// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PIDCommand.h>
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
  bool isGamePieceInClaw();
  bool checkEdge();

  enum armVals { armDown = -1, armOff = 0, armUp = 1 };

  enum wristVals { wristDown = -1, wristOff = 0, wristUp = 1 };

  enum shoulderVals { shoulderDown = -1, shoulderOff = 0, shoulderUp = 1 };

  void appendageReset(bool isPneumaticsIn);
  bool isSensorWorking(rev::CANSparkMax* canMotor,
                       rev::RelativeEncoder* canEncoder, double last);
  bool isSensorWorking(rev::CANSparkMax* canMotor, frc::Encoder* frcEncoder,
                       double last);
  bool isSensorWorking(frc::AnalogInput* aInput, double last);
  bool getArmWorking();
  bool getShoulderWorking();
  bool getWristWorking();
  bool getAnalogWorking();

  void frontClawPneumaticsIn();
  void frontClawPneumaticsOut();
  void backClawPneumaticsIn();
  void backClawPneumaticsOut();
  double getClaw1();
  double getClaw2();

  bool getArmExtended();

  bool unleashThePower = false;

 private:
  int m_frontRollerId = 15;
  int m_backRollerId = 14;
  int m_armId = 16;
  int m_shoulderId = 17;
  // roller
  int p_Roller1Id_a = 1;
  int p_Roller1Id_b = 0;
  int p_RollerB_Id_a = 3;
  int p_RollerB_Id_b = 2;
  int p_RollerF_Id_a = 4;
  int p_RollerF_Id_b = 5;

  int m_wristMotorId = 18;
  int p_pcmId = 19;

  // Appendage Encoder Limits
  double shoulder_max = 0.0;
  double shoulder_min = -2000.0;

  double arm_max = -5;
  double arm_min = -220.0;

  double wrist_max = 3000.0;
  double wrist_min = 0.0;

  // Claw Motors
  rev::CANSparkMax* m_backRollerMotor;
  rev::CANSparkMax* m_frontRollerMotor;

  // Telescoping Arm Motor
  rev::CANSparkMax* m_armMotor;
  rev::RelativeEncoder* arm_Encoder;

  // Shoulder Motor
  rev::CANSparkMax* m_shoulderMotor;
  frc::Encoder* shoulder_Encoder;

  // Wrist Motor
  rev::CANSparkMax* m_wristMotor;
  frc::Encoder* wrist_Encoder;

  // Claw Rear Roller Cylinder
  frc::DoubleSolenoid* p_backRollerCylinder1;
  frc::DoubleSolenoid* p_backCSCylinder;
  frc::DoubleSolenoid* p_frontCSCylinder;

  // Claw UltSnd
  frc::AnalogInput* claw1_a_input;
  frc::AnalogInput* claw2_a_input;
  // edge on chassis
  frc::AnalogInput* edge1_a_input;
  frc::AnalogInput* edge2_a_input;

  double remapVal(double i, double threshold);
  double deadband(double i, double threshold);
  double analogToDistance(double i);

  frc2::PIDController Arm_PIDController{1.0, 0, 0};
  frc2::PIDController Shoulder_PIDController{1.0, 0, 0};
  frc2::PIDController Wrist_PIDController{1.0, 0, 0};

  double lastArm = 0;
  double lastShoulder = 0;
  double lastWrist = 0;
  double lastClaw1 = 0;
  double lastClaw2 = 0;
};
