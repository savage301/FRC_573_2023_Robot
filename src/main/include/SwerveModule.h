// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "ctre/Phoenix.h"
#include "def.h"
#include "pid.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderChannel);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
  void SetDesiredState(const frc::SwerveModuleState& state);
  void resetTurningMotorHeading();
  void switchIdleMode(bool auton);

 private:
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  ctre::phoenix::sensors::CANCoder m_turningEncoder;
};
