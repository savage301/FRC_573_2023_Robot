// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "ctre/Phoenix.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderChannel);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kDriveEncoderResolution = 6.75;
  static constexpr int kSteerEncoderResolution = 4096;

  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 10_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  ctre::phoenix::sensors::CANCoder m_turningEncoder;

  frc2::PIDController m_drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      12 / 1.5,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};
};
