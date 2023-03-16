// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/trajectory/TrajectoryConfig.h>

#include <numbers>
#include <vector>

#include "SwerveModule.h"
#include "def.h"
#include "pid.h"

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
  void UpdateOdometry(frc::Pose2d camerapose);

  void DriveWithJoystick(double xJoy, double yJoy, double rJoy,
                         bool fieldRelative, bool lim, bool gyrostablize);

  void ResetOdometry(const frc::Pose2d& pose);
  frc::ChassisSpeeds GetRobotVelocity();
  bool isBlue = false;

  frc::Pose2d GetPose() const;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::dimensionless::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::dimensionless::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::dimensionless::scalar> m_rotLimiter{3 / 1_s};

  frc::TrajectoryConfig auto_traj{kMaxSpeed, kMaxAcceleration};

  void setTrajCon();

  void autoBalance();

  // bool onRamp = false;

  enum RampPos {
    floor = 0,
    upward = 1,
    balanced = 2,
    downside = 3,
    floorback = 4
  };

  int currRampPos;

  int lastRampSide;

  bool crossedramp;

  int rampState;

  void pumpOutSensorVal();

  bool isGyroWorking();

  void resetGyro(double angle);

  void updateGyroAngle();

  double gryoStablize();
  double gyroSetpoint;

  void resetDrivetrain(bool auton);
  void stopDrivetrain(bool gyro, double r);

 private:
  frc::Translation2d m_frontLeftLocation{+0.3175_m, +0.27305_m};
  frc::Translation2d m_frontRightLocation{+0.3175_m, -0.27305_m};
  frc::Translation2d m_backLeftLocation{-0.3175_m, +0.27305_m};
  frc::Translation2d m_backRightLocation{-0.3175_m, -0.27305_m};

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

  double last = 0;

  int counter = 0;

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      frc::Pose2d{0_m, 0_m, 0_deg},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 0.1}};
};
