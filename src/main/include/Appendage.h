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
  bool getArmWorking();
  bool getShoulderWorking();
  bool getWristWorking();
  void clawPneumaticsIn();
  void clawPneumaticsOut();

 private:
  int m_frontRollerId = 15;
  int m_backRollerId = 14;
  int m_armId = 16;
  int m_shoulderId = 17;
  // roller
  int p_Roller1Id_a = 0;
  int p_Roller1Id_b = 1;
  int p_Roller2Id_a = 2;
  int p_Roller2Id_b = 3;
  // Charge station claw
  int p_claw1Id_a = 4;
  int p_claw1Id_b = 5;
  int p_claw2Id_a = 6;
  int p_claw2Id_b = 7;
  int m_wristMotorId = 18;
  int p_pcmId = 19;

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
  frc::DoubleSolenoid* p_backRollerCylinder2;

  // Charge Station Claw Cylinders
  frc::DoubleSolenoid* p_clawCylinder1;
  frc::DoubleSolenoid* p_clawCylinder2;

  // Claw UltSnd
  frc::AnalogInput* claw1_a_input;

  // edge on chassis
  frc::AnalogInput* edge1_a_input;
  frc::AnalogInput* edge2_a_input;

  double remapVal(double i, double threshold);

  frc2::PIDController Arm_PIDController{1.0, 0, 0};
  frc2::PIDController Shoulder_PIDController{1.0, 0, 0};
  frc2::PIDController Wrist_PIDController{1.0, 0, 0};

  double lastArm = 0;
  double lastShoulder = 0;
  double lastWrist = 0;
};
