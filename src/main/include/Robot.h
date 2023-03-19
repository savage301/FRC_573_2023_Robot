// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Compressor.h>
#include <frc/DataLogManager.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/controller/BangBangController.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/RamseteController.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <wpi/DataLog.h>

#include <memory>
#include <span>
#include <string>
#include <vector>

#include "Appendage.h"
#include "Drivetrain.h"
#include "Led.h"
#include "auto.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "pid.h"
#include "poses.h"

class Robot : public frc::TimedRobot {
 public:
  Drivetrain m_swerve;
  Appendage m_appendage;
  Led m_leds;
  frc::XboxController m_controller1{0};
  frc::XboxController m_controller2{1};
  frc::XboxController m_controllerTest{2};

  // -------------- Added for Auto------------------------------
  pathplanner::PathPlannerTrajectory trajectoryPP_;
  frc::Trajectory trajectory_;
  // The timer to use during the autonomous period.
  frc::Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  frc::Field2d m_field;
  frc::Field2d field_off;

  // Robot
  frc2::PIDController X_PIDController{-0.005, 0, 0};
  frc2::PIDController Y_PIDController{0.005, 0, 0};
  frc::ProfiledPIDController<units::radians> theta_PIDController{
      3, 0.0, 0.0, {kMaxAngularSpeed, kMaxAngularAccel}};

  // Swerve Controller to follow the trajectory
  frc::HolonomicDriveController m_holonmicController =
      frc::HolonomicDriveController(X_PIDController, Y_PIDController,
                                    theta_PIDController);

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
  std::string m_autoSelected;

  nt::DoubleArraySubscriber botPose;
  nt::IntegerSubscriber validTarget;
  nt::DoubleArraySubscriber cornerXy;

  frc::Compressor compressor =
      frc::Compressor(19, frc::PneumaticsModuleType::CTREPCM);

  int autoState;
  bool firstTime;

  std::shared_ptr<nt::NetworkTable> table;

  enum Grid { humanLeft, humanCenter, humanRight };

  int tarGrid;

  bool isBlue;

  enum GamePiece { cone = 1, cube = 2 };

  int tarGamePiece;

  bool hasGamePiece;

  enum fA_Pos { top, left, right, bot };
  int curFA_Pos;
  int curFA_Pos_Latch;
  frc::Pose2d offPose = frc::Pose2d(
      frc::Translation2d(units::meter_t(-7.99), units::meter_t(-4.105)),
      frc::Rotation2d(units::degree_t(0)));

  pathplanner::PathPlannerTrajectory pathGenerate(int slot);
  pathplanner::PathPlannerTrajectory pathGenerate(frc::Pose2d tarPose,
                                                  frc::Rotation2d angle);
  pathplanner::PathPlannerTrajectory pathGenerate(frc::Pose2d startPose,
                                                  frc::Pose2d tarPose,
                                                  frc::Rotation2d angle);
  pathplanner::PathPlannerTrajectory pathLoad(std::string path);
  void driveWithTraj(pathplanner::PathPlannerTrajectory trajectoryPP_,
                     frc::Pose2d offPose);
  void driveWithTraj(bool auton);
  void autonomousPaths(int select);

  void handleLedModes(bool isGamePiece, bool isGamePieceAcquired,
                      int tarGamePiece, bool isEdgeClose);
  void getPowerDistribution();
  void selectGamePiece();
  void selectScoringGrid();
  void autonomousPaths(bool isBlue, int slot, frc::Pose2d poseMidPoint,
                       frc::Pose2d poseCube);
  bool isPassCenterLine();

  void EstimatePose();
  void EstimatePose(int camera_pipline);
  double estimateGamePieceDistanceToCenter();

  void driveToCS(bool isBlue);
  void driveToCSsimple(bool isBlue);
  void basicAuto(bool isBlue);
  void basicAuto2();
  void updateHasGamePiece();
  void twoGPAuto();
};
