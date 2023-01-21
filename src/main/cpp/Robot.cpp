// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  botPose = table->GetDoubleArrayTopic("botpose").Subscribe({});
  validTarget = table->GetIntegerTopic("tv").Subscribe({});
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  /*m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/

// ---------------------------------- Trajectory Following Auto Section ---------------------
  // Generate trajectory to follow for autonomous
  // Start the timer.
    m_timer.Start();

    // Send Field2d to SmartDashboard.
    frc::SmartDashboard::PutData(&m_field);

    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    m_swerve.ResetOdometry(exampleTrajectory.InitialPose());

    // Send our generated trajectory to Field2d.
    m_field.GetObject("traj")->SetTrajectory(exampleTrajectory);
// ----------------------------------------------------------------------------------------

}

void Robot::AutonomousPeriodic() {
  /*if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/

// ---------------------------------- Trajectory Following Auto Section ---------------------
  // Update odometry.
    m_swerve.UpdateOdometry();

  // Update robot position on Field2d.
    m_field.SetRobotPose(m_swerve.GetPose());

  // Send Field2d to SmartDashboard.
    frc::SmartDashboard::PutData(&m_field);

  if (m_timer.Get() < exampleTrajectory.TotalTime()) {
  // Get the desired pose from the trajectory.
    auto desiredPose = exampleTrajectory.Sample(m_timer.Get());

  // Get the reference chassis speeds from the Ramsete Controller.
    auto refChassisSpeeds =
      m_ramseteController.Calculate(m_swerve.GetPose(), desiredPose);

  // Set the linear and angular speeds.
    m_swerve.Drive(refChassisSpeeds.vx, refChassisSpeeds.vy, refChassisSpeeds.omega,false);
    } 
    else {
    m_swerve.Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(0), false); 
    }
  // ----------------------------------------------------------------------------------------
    
}

void Robot::TeleopInit() {

  m_swerve.ResetOdometry(frc::Pose2d{5_m, 5_m, 0_rad});
}

void Robot::TeleopPeriodic(){

  // Drive with joystick 0 with swervedrive
  m_swerve.DriveWithJoystick(m_controller.GetLeftY(),m_controller.GetLeftX(),m_controller.GetRightX(),true);

  int validTarFnd = validTarget.Get();
  std::vector<double> robotPose = botPose.Get();
  if (validTarFnd == 1 && robotPose.size()>0) {
      
          
          frc::SmartDashboard::PutNumber("robotPoseX",robotPose[0]);
          frc::SmartDashboard::PutNumber("robotPoseY",robotPose[1]);
          frc::SmartDashboard::PutNumber("robotPoseYaw",robotPose[5]);

          frc::Translation2d tmp2d = frc::Translation2d(units::meter_t(robotPose[0]), units::meter_t(robotPose[1]));
          frc::Rotation2d tmpAng = frc::Rotation2d(units::degree_t(robotPose[5]));
          frc::Pose2d fldPose = frc::Pose2d(tmp2d, tmpAng);

          m_field.SetRobotPose(fldPose);
          m_swerve.ResetOdometry(fldPose);

  } else {
  // ----------- Update robot pose and send it to field object on DS ----------------------------- 
    // Update robot position on Field2d.
      m_swerve.UpdateOdometry();
      m_field.SetRobotPose(m_swerve.GetPose());
  // ----------------------------------------------------------------------------------------
  }

  // Send Field2d to SmartDashboard.
  frc::Pose2d offPose = frc::Pose2d(frc::Translation2d(units::meter_t(-7.99),units::meter_t(-4.105)), frc::Rotation2d(units::degree_t(0)));

  //frc::SmartDashboard::PutData(&m_field);
  
  field_off.SetRobotPose(m_field.GetRobotPose().RelativeTo(offPose));
  frc::SmartDashboard::PutData(&field_off);

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
