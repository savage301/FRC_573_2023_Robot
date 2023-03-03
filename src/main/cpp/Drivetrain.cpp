// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  frc::SwerveModulePosition FL = {m_frontLeft.GetPosition().distance,
                                  m_frontLeft.GetPosition().angle};
  frc::SwerveModulePosition FR = {m_frontRight.GetPosition().distance,
                                  m_frontRight.GetPosition().angle};
  frc::SwerveModulePosition BL = {m_backLeft.GetPosition().distance,
                                  m_backLeft.GetPosition().angle};
  frc::SwerveModulePosition BR = {m_backLeft.GetPosition().distance,
                                  m_backRight.GetPosition().angle};

  m_poseEstimator.Update(m_gyro.GetRotation2d(), {FL, FR, BL, BR});

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  // m_poseEstimator.AddVisionMeasurement(
  //    ExampleGlobalMeasurementSensor::GetEstimatedGlobalPose(
  //        m_poseEstimator.GetEstimatedPosition()),
  //    frc::Timer::GetFPGATimestamp() - 0.3_s);
}

void Drivetrain::UpdateOdometry(frc::Pose2d camerapose) {
  frc::SwerveModulePosition FL = {m_frontLeft.GetPosition().distance,
                                  m_frontLeft.GetPosition().angle};
  frc::SwerveModulePosition FR = {m_frontRight.GetPosition().distance,
                                  m_frontRight.GetPosition().angle};
  frc::SwerveModulePosition BL = {m_backLeft.GetPosition().distance,
                                  m_backLeft.GetPosition().angle};
  frc::SwerveModulePosition BR = {m_backLeft.GetPosition().distance,
                                  m_backRight.GetPosition().angle};

  m_poseEstimator.Update(m_gyro.GetRotation2d(), {FL, FR, BL, BR});

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  m_poseEstimator.AddVisionMeasurement(camerapose,
                                       frc::Timer::GetFPGATimestamp() - 0.3_s);
}

frc::ChassisSpeeds Drivetrain::GetRobotVelocity() {
  return m_kinematics.ToChassisSpeeds(
      {m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(),
       m_backRight.GetState()});
}

void Drivetrain::DriveWithJoystick(double xJoy, double yJoy, double rJoy,
                                   bool fieldRelative, bool lim) {
  // Weikai: add bool lim to limit speed to 50%

  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed =
      Drivetrain::m_xspeedLimiter.Calculate(frc::ApplyDeadband(xJoy, 0.02)) *
      Drivetrain::kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed =
      Drivetrain::m_yspeedLimiter.Calculate(frc::ApplyDeadband(yJoy, 0.02)) *
      Drivetrain::kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const auto rot =
      Drivetrain::m_rotLimiter.Calculate(frc::ApplyDeadband(rJoy, 0.02)) *
      Drivetrain::kMaxAngularSpeed;

  Drive(lim ? xSpeed / 2 : xSpeed, lim ? ySpeed / 2 : ySpeed,
        lim ? rot / 2 : rot, fieldRelative);
  // UpdateOdometry();
  frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_poseEstimator.ResetPosition(
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}

frc::Pose2d Drivetrain::GetPose() const {
  return m_poseEstimator.GetEstimatedPosition();
}

void Drivetrain::setTrajCon() {
  auto_traj.SetKinematics(m_kinematics);
}

void Drivetrain::autoBalance() {
  double gV[3];
  // change these two
  double RampZ = .7;
  double balancedZ = 0.9;
   units::meters_per_second_t fastSpeed = 1.5_mps;
   units::meters_per_second_t slowSpeed = 1_mps;
   units::meters_per_second_t zeroSpeed = 0_mps; 
   units::radians_per_second_t zeroSpeedrot{0}; 
  if (m_gyro.GetGravityVector(gV) == ctre::phoenix::ErrorCode::OK) {
    // vector towards the ground
    // frc::SmartDashboard::PutNumber("GV Gravity Vector X", gV[0]);
    // frc::SmartDashboard::PutNumber("GV Gravity Vector Y", gV[1]);
    frc::SmartDashboard::PutNumber("GV Gravity Vector Z", gV[2]);
  }
// -----Cross Ramp Section ---------------------------

  if(currRampPos == Drivetrain::RampPos::floor && abs(gV[2]) > RampZ && !crossedramp){
    Drive(-fastSpeed, zeroSpeed, zeroSpeedrot, true);
      currRampPos = Drivetrain::RampPos::floor;
  }
  else if (currRampPos == Drivetrain::RampPos::floor && abs(gV[2]) < RampZ && !crossedramp){
    currRampPos = Drivetrain::RampPos::upward;
    lastRampSide = Drivetrain::RampPos::upward;
    Drive(-fastSpeed, zeroSpeed, zeroSpeedrot, true);
  }

  else if (currRampPos == Drivetrain::RampPos::upward && abs(gV[2]) > balancedZ && !crossedramp){
    currRampPos = Drivetrain::RampPos::balanced;
    Drive(-fastSpeed, zeroSpeed, zeroSpeedrot, true);
  }

  else if (currRampPos == Drivetrain::RampPos::balanced && abs(gV[2]) < RampZ && !crossedramp){
    currRampPos = Drivetrain::RampPos::downside;
    lastRampSide = Drivetrain::RampPos::downside;
    Drive(-fastSpeed, zeroSpeed, zeroSpeedrot, true);
  }

  else if (currRampPos == Drivetrain::RampPos::downside && abs(gV[2]) > balancedZ && !crossedramp){
    currRampPos = Drivetrain::RampPos::floorback;
    Drive(fastSpeed, zeroSpeed, zeroSpeedrot, true);
    crossedramp = true;
  }

  // --------------------------------------------------------------------------------
  // - Balance Section --------------------------------------------
  else if (currRampPos == Drivetrain::RampPos::floorback && abs(gV[2]) < RampZ && crossedramp){
    currRampPos = Drivetrain::RampPos::downside;
    lastRampSide = Drivetrain::RampPos::downside;
    Drive(slowSpeed, zeroSpeed, zeroSpeedrot, true);
  }

  else if ((currRampPos == Drivetrain::RampPos::downside || currRampPos == Drivetrain::RampPos::upward) && abs(gV[2]) > balancedZ && crossedramp){
    currRampPos = Drivetrain::RampPos::balanced;
    Drive(zeroSpeed, zeroSpeed, zeroSpeedrot, true);
  }

  else if (currRampPos == Drivetrain::RampPos::balanced && abs(gV[2]) < RampZ && crossedramp && lastRampSide == Drivetrain::RampPos::downside){
    currRampPos = Drivetrain::RampPos::upward;
    lastRampSide = Drivetrain::RampPos::upward;
    Drive(-slowSpeed, zeroSpeed, zeroSpeedrot, true);
  }
   else if (currRampPos == Drivetrain::RampPos::balanced && abs(gV[2]) < RampZ && crossedramp && lastRampSide == Drivetrain::RampPos::upward){
    currRampPos = Drivetrain::RampPos::downside;
    lastRampSide = Drivetrain::RampPos::downside;
    Drive(slowSpeed, zeroSpeed, zeroSpeedrot, true);
  }
}

#include <frc/smartdashboard/SmartDashboard.h>
#define pumpOut frc::SmartDashboard::PutNumber
void Drivetrain::pumpOutSensorVal() {
  double curGyro = m_gyro.GetAngle();
  double gV[3];

  pumpOut("Gyro angle", curGyro);
  if (m_gyro.GetGravityVector(gV) == ctre::phoenix::ErrorCode::OK)
    pumpOut("GV Gravity Vector Z", gV[2]);
}

bool Drivetrain::isGyroWorking() {
  double cur;
  cur = m_gyro.GetAngle();
  if (cur != last) {  // current is not equal to previous
    last = cur;
    return true;
  }
  last = cur;
  return false;
}

void Drivetrain::resetGyro() {
  m_gyro.SetYaw(0, 0);
}
