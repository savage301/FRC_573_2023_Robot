// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,
                     rev::CANSparkMax::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder(
          rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
      m_turningEncoder(turningEncoderChannel) {
  m_driveEncoder.SetPositionConversionFactor(
      2 * Pi * kWheelRadius / kDriveGearRatio);  // Conversion from rot to m
  m_driveEncoder.SetVelocityConversionFactor(
      (2 * Pi * kWheelRadius / kDriveGearRatio) /
      60);  // Converstion from rpm to to m/s

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t{Pi},
                                               units::radian_t{Pi});
  //  m_driveMotor.SetOpenLoopRampRate(0.1);
  m_driveMotor.SetSmartCurrentLimit(40);
  m_turningMotor.SetSmartCurrentLimit(40);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetAbsolutePosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{((m_driveEncoder.GetPosition()))},
          units::radian_t{m_turningEncoder.GetAbsolutePosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{m_turningEncoder.GetPosition()});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetAbsolutePosition()},
      state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs. Flip directions as needed to get modules to spin
  // proper direction.

  // inverted motor id 1, 2, and 8 thru rev hw client via flipping kInverted in
  // advanced
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);

  // 1, 4. 7. 10
  int driveMotorId = m_driveMotor.GetDeviceId();
  switch (driveMotorId) {
    case 1:
      pumpOutNum("Drive Encoder FL", m_driveEncoder.GetPosition());
      pumpOutNum("Turn Encoder FL", m_turningEncoder.GetAbsolutePosition());
      pumpOutNum("fl vol output", driveOutput);
      pumpOutNum("fl measured speed", m_driveEncoder.GetVelocity());
      /*frc::SmartDashboard::PutNumber(
          "DrivePCF",
          m_driveEncoder
              .GetPositionConversionFactor());  // temp to just confirm should
                                                // be ~0.04726
      frc::SmartDashboard::PutNumber(
          "DriveVCF",
          m_driveEncoder
              .GetVelocityConversionFactor());  // temp to just confirm should
                                                // be ~0.00078771358
       */
      break;
    case 4:
      pumpOutNum("Drive Encoder FR", m_driveEncoder.GetPosition());
      pumpOutNum("Turn Encoder FR", m_turningEncoder.GetAbsolutePosition());
      break;
    case 7:
      pumpOutNum("Drive Encoder BL", m_driveEncoder.GetPosition());
      pumpOutNum("Turn Encoder BL", m_turningEncoder.GetAbsolutePosition());
      break;
    case 10:
      pumpOutNum("Drive Encoder BR", m_driveEncoder.GetPosition());
      pumpOutNum("Turn Encoder BR", m_turningEncoder.GetAbsolutePosition());
      break;

    default:
      break;
  }
}

void SwerveModule::resetTurningMotorHeading() {
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetAbsolutePosition()}, 0_rad);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput});
}

void SwerveModule::switchIdleMode(bool auton) {
  m_driveMotor.SetIdleMode(auton ? rev::CANSparkMax::IdleMode::kBrake
                                 : rev::CANSparkMax::IdleMode::kCoast);
}
