// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>



SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel,rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,rev::CANSparkMax::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
      m_turningEncoder(turningEncoderChannel) {


  m_driveEncoder.SetPositionConversionFactor(2 * std::numbers::pi * kWheelRadius /
                                     kDriveEncoderResolution);
  m_driveEncoder.SetVelocityConversionFactor(2 * std::numbers::pi * kWheelRadius /
                                     kDriveEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetPosition()}};
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
      units::radian_t{m_turningEncoder.GetPosition()}, state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

 // Set the motor outputs.
  if (m_driveMotor.GetDeviceId() == 1 || m_driveMotor.GetDeviceId() == 7 || m_driveMotor.GetDeviceId() == 4){
    m_driveMotor.SetVoltage(units::volt_t{-driveOutput} );//+ driveFeedforward);
  }
  else{
    m_driveMotor.SetVoltage(units::volt_t{driveOutput} );//+ driveFeedforward);
  }
  //m_driveMotor.SetVoltage(units::volt_t{driveOutput} );//+ driveFeedforward);
  
  if (m_turningMotor.GetDeviceId() == 5 || m_turningMotor.GetDeviceId() == 11){
    m_turningMotor.SetVoltage(units::volt_t{-turnOutput} );//+ turnFeedforward);
  }else{
    m_turningMotor.SetVoltage(units::volt_t{turnOutput} );//+ turnFeedforward);
  }
}