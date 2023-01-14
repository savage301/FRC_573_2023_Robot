// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>


#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/trajectory/TrajectoryConfig.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();

  void DriveWithJoystick(double xJoy, double yJoy, double rJoy, bool fieldRelative);

  void ResetOdometry(const frc::Pose2d& pose);

  frc::Pose2d GetPose() const;

  static constexpr units::meters_per_second_t kMaxSpeed =
      1_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second

  static constexpr auto kMaxAcceleration =
     units::meters_per_second_squared_t(2.5);  // meters per second^2

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    frc::SlewRateLimiter<units::dimensionless::scalar> m_xspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::dimensionless::scalar> m_yspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::dimensionless::scalar> m_rotLimiter{3 / 1_s};

    frc::TrajectoryConfig auto_traj {kMaxSpeed,kMaxAcceleration};

 private:
  frc::Translation2d m_frontLeftLocation{+0.305_m, +0.305_m};
  frc::Translation2d m_frontRightLocation{+0.305_m, -0.305_m};
  frc::Translation2d m_backLeftLocation{-0.305_m, +0.305_m};
  frc::Translation2d m_backRightLocation{-0.305_m, -0.305_m};

  SwerveModule m_frontLeft{1, 2, 3};
  SwerveModule m_frontRight{4, 5, 6};
  SwerveModule m_backLeft{7, 8, 9};
  SwerveModule m_backRight{10, 11, 12};

  ctre::phoenix::sensors::WPI_Pigeon2 m_gyro{13};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};



};