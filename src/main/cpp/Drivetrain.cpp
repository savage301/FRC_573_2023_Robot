// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>


#include "Drivetrain.h"

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
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::DriveWithJoystick(double xJoy, double yJoy, double rJoy, bool fieldRelative, bool lim) {
    // Weikai: add bool lim to limit speed to 50%

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -Drivetrain::m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(xJoy, 0.02)) *
                         Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -Drivetrain::m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(yJoy, 0.02)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -Drivetrain::m_rotLimiter.Calculate(
                         frc::ApplyDeadband(rJoy, 0.02)) *
                     Drivetrain::kMaxAngularSpeed;

    
    Drive(lim ? xSpeed/2:xSpeed, lim ? ySpeed/2 : ySpeed, lim ? rot/2 : rot, fieldRelative);
    //UpdateOdometry();
    frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
  }


  void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                            {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_backLeft.GetPosition(), m_backRight.GetPosition()},
                            pose);

  }

  frc::Pose2d Drivetrain::GetPose() const {
  return m_odometry.GetPose();
  }

  void Drivetrain::setTrajCon(){
    auto_traj.SetKinematics(m_kinematics);
  }

