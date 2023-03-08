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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

class Robot : public frc::TimedRobot {
 public:
  Drivetrain m_swerve;
  Appendage m_appendage;
  Led m_leds;
  frc::XboxController m_controller1{0};
  frc::XboxController m_controller2{1};

  // -------------- Added for Auto------------------------------
  pathplanner::PathPlannerTrajectory trajectoryPP_;
  frc::Trajectory trajectory_;
  // The timer to use during the autonomous period.
  frc::Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  frc::Field2d m_field;
  frc::Field2d field_off;

  frc2::PIDController X_PIDController{1.0, 0, 0};
  frc2::PIDController Y_PIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> theta_PIDController{
      1, 0.0, 0.0, {m_swerve.kMaxAngularSpeed, m_swerve.kMaxAngularAccel}};

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
  const std::string kAutonPaths7 = "Basic Red Right Right";
  const std::string kAutonPaths8 = "Basic Red Left Left";
  const std::string kAutonPaths9 = "Basic Blue Right Right";
  const std::string kAutonPaths10 = "Basic Blue Left Left";
  const std::string kAutonPaths11 = "make a turn";
  const std::string kAutonPaths5 = "Red Left 3 to CS";
  const std::string kAutonPaths6 = "Blue Left 3 to CS";
  const std::string kAutonPaths1 = "Red Right 6 to 7";
  const std::string kAutonPaths2 = "Red Left 2 to 1";
  const std::string kAutonPaths3 = "Blue Right 6 to 7";
  const std::string kAutonPaths4 = "Blue Left 2 to 1";
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  nt::DoubleArraySubscriber botPose;
  nt::IntegerSubscriber validTarget;
  nt::DoubleArraySubscriber cornerXy;

  frc::Compressor compressor =
      frc::Compressor(19, frc::PneumaticsModuleType::CTREPCM);

  int autoState;
  bool firstTime;

  std::shared_ptr<nt::NetworkTable> table;

#define pose1(x, y) frc::Pose2d(x, y, frc::Rotation2d(0_deg))
#define pose2(x, y) frc::Pose2d(x, y, frc::Rotation2d(180_deg))
#define poseCubes(x, y) \
  frc::Pose2d(x, y, frc::Rotation2d(180_deg))  // facing away from the wall
#define poseRed(y) pose1(6.41_m, y)
#define poseBlue(y) pose2(-6.41_m, y)
#define poseRedMid(y) \
  pose1(5.09_m, y)  // not sure 'bout this, might be 180, i.e. pose2
#define poseBlueMid(y) pose1(-5.09_m, y)
#define poseRedCubes_(y) poseCubes(1.2_m, y)
#define poseBlueCubes_(y) poseCubes(-1.2_m, y)

  std::vector<frc::Pose2d> redPose = {                     // slot num
                                      poseRed(-3.5_m),     // 0
                                      poseRed(-2.94_m),    // 1
                                      poseRed(-2.38_m),    // 2
                                      poseRed(-1.82_m),    // 3
                                      poseRed(-1.26_m),    // 4
                                      poseRed(-.7_m),      // 5
                                      poseRed(-.14_m),     // 6
                                      poseRed(.42_m),      // 7
                                      poseRed(.98_m)};     // 8
  std::vector<frc::Pose2d> bluePose = {                    // slot num
                                       poseBlue(.98_m),    // 0
                                       poseBlue(.42_m),    // 1
                                       poseBlue(-.14_m),   // 2
                                       poseBlue(-.7_m),    // 3
                                       poseBlue(-1.26_m),  // 4
                                       poseBlue(-1.82_m),  // 5
                                       poseBlue(-2.38_m),  // 6
                                       poseBlue(-2.94_m),  // 7
                                       poseBlue(-3.5_m)};  // 8

  frc::Pose2d redRightMidPose = poseRedMid(0.67_m);
  frc::Pose2d redLeftMidPose = poseRedMid(-3.17_m);
  frc::Pose2d redRightcube = poseRedCubes_(.57_m);
  frc::Pose2d redLeftcube = poseRedCubes_(-3.09_m);

  frc::Pose2d blueLeftMidPose = poseBlueMid(0.67_m);
  frc::Pose2d blueRightMidPose = poseBlueMid(-3.17_m);
  frc::Pose2d blueLeftcube = poseBlueCubes_(.57_m);
  frc::Pose2d blueRightcube = poseBlueCubes_(-3.09_m);

  frc::Pose2d redCharge = pose1(5.88_m, -1.29_m);
  frc::Pose2d blueCharge = pose1(-5.88_m, -1.29_m);
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
};
