// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/Timer.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include "Drivetrain.h"
#include "Appendage.h"
#include "Led.h"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>

#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPoint.h>

#include <frc/controller/HolonomicDriveController.h>

#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

#include <frc/controller/BangBangController.h>

class Robot : public frc::TimedRobot {
 public:
  Drivetrain m_swerve;
  Appendage m_appendage;
  frc::XboxController m_controller1{0};
  frc::XboxController m_controller2{1};

// -------------- Added for Auto------------------------------
  frc::Trajectory exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(5_m, 5_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(6_m, 6_m), frc::Translation2d(7_m, 4_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(8_m, 5_m, frc::Rotation2d(0_deg)),
      // Pass the config
      m_swerve.auto_traj);

  pathplanner::PathPlannerTrajectory trajectoryPP_;
  frc::Trajectory trajectory_;
  // The timer to use during the autonomous period.
  frc::Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  frc::Field2d m_field;

  frc::Field2d field_off;

  // The Ramsete Controller to follow the trajectory.
  frc::RamseteController m_ramseteController;

  frc2::PIDController X_PIDController{1.0, 0, 0}; 
  frc2::PIDController Y_PIDController{1.0, 0, 0}; 
  frc::ProfiledPIDController<units::radians> theta_PIDController{
      1,
      0.0,
      0.0,
      {m_swerve.kMaxAngularSpeed, m_swerve.kMaxAngularAccel}};

 //Swerve Controller to follow the trajectory
  frc::HolonomicDriveController m_holonmicController = frc::HolonomicDriveController(X_PIDController,Y_PIDController,theta_PIDController);

// -------------------------------------------------------------

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  nt::DoubleArraySubscriber botPose;  
  nt::IntegerSubscriber validTarget;
  nt::DoubleArraySubscriber cornerXy;

  int autoState = 0;
  bool firstTime;

  std::shared_ptr<nt::NetworkTable> table;

#define pose1(x, y) frc::Pose2d(x, y, frc::Rotation2d(0_deg))
#define poseRed(y) pose1(6.84_m,y)
#define poseBlue(y) pose1(-6.84_m,y)

  std::vector<frc::Pose2d> redPose = { // slot num
      poseRed(-3.5_m), // 0
      poseRed(-2.94_m), // 1
      poseRed(-2.38_m), // 2
      poseRed(-1.82_m), // 3
      poseRed(-1.26_m), // 4
      poseRed(-.7_m), // 5
      poseRed(-.14_m), // 6
      poseRed(.42_m), // 7
      poseRed(.98_m)}; // 8
  std::vector<frc::Pose2d> bluePose = { // slot num
      poseBlue(.98_m), // 0
      poseBlue(.42_m), // 1
      poseBlue(-.14_m), // 2
      poseBlue(-.7_m), // 3
      poseBlue(-1.26_m), // 4
      poseBlue(-1.82_m), // 5
      poseBlue(-2.38_m), // 6
      poseBlue(-2.94_m), // 7
      poseBlue(-3.5_m)}; // 8

  enum Grid
  {
    humanLeft,
    humanCenter,
    humanRight
  };

  int tarGrid;

  bool isBlue;

  enum GamePiece
  {
    cone = 1,
    cube = 2
  };

  int tarGamePiece;

  bool hasGamePiece;

  enum fA_Pos {
    top,
    left,
    right,
    bot
  };
  int curFA_Pos;
  int curFA_Pos_Latch;

  void pathGenerate(int slot, frc::Pose2d offPose);
  void driveWithTraj();
  void autonomousPaths(int select);
};
