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

class Robot : public frc::TimedRobot {
 public:
  Drivetrain m_swerve;
  Appendage m_appendage;
  frc::XboxController m_controller{0};

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

  // The timer to use during the autonomous period.
  frc::Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  frc::Field2d m_field;
  
  frc::Field2d field_off;

  // The Ramsete Controller to follow the trajectory.
  frc::RamseteController m_ramseteController;

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
};
