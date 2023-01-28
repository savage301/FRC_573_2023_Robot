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

  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  botPose = table->GetDoubleArrayTopic("botpose").Subscribe({});
  validTarget = table->GetIntegerTopic("tv").Subscribe({});
  cornerXy = table->GetDoubleArrayTopic("tcornxy").Subscribe({});


  table->PutNumber("ledMode", 0);
  table->PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//
      
  table -> PutNumber("pipeline", 0);
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
  hasGamePiece = false;
  isBlue = false;
  tarGrid = Grid::humanLeft;
  m_swerve.ResetOdometry(frc::Pose2d{5_m, 5_m, 0_rad});
}

void Robot::TeleopPeriodic(){
  frc::AnalogInput a_Input = frc::AnalogInput(0);
  frc::SmartDashboard::PutNumber("AnalogInput", a_Input.GetValue());

  int validTarFnd = validTarget.Get();

  if (validTarFnd)
  {
    std::vector<double> coneCornerXy = cornerXy.Get();
    std::vector<bool> x_orien, y_orien;
    int length = coneCornerXy.size();
    frc::SmartDashboard::PutNumber("Corner Arr Length", length);
    std::vector<double> avg = {0.0, 0.0};
    int i;
    int idxFA = 0; // furtherest away
    double FADist =0;
    for (i = 0; i < length; i += 2)
    {
      avg[0] += coneCornerXy[i];
      avg[1] += coneCornerXy[i + 1];
    }
    avg[0] /= (.5 * i);
    avg[1] /= (.5 * i);
    frc::SmartDashboard::PutNumber("avg x", avg[0]);
    frc::SmartDashboard::PutNumber("avg y", avg[1]);
    for (i = 0; i < length; i += 2)
    {
      double tmpDistFA = sqrt(pow((coneCornerXy[i] - avg[0]),2) + pow((coneCornerXy[i + 1] - avg[1]),2));
      if (tmpDistFA >= FADist){
        FADist = tmpDistFA;
        idxFA = i;
      }
    }
    frc::SmartDashboard::PutNumber("index of FA", idxFA);
    for (i = 0; i < length; i += 2)
    {
      double x, y;
      if (i != idxFA)
      {
        x = coneCornerXy[i] - coneCornerXy[idxFA];
        y = coneCornerXy[i + 1] - coneCornerXy[idxFA + 1];
        
        x_orien.push_back(x>0);
        y_orien.push_back(y>0);
      }
    }
    // if x all true, right
    // x all false, left
    // y all false, bot
    /*for (bool y_ : y_orien) {
      if (y_)
        curFA_Pos = Robot::fA_Pos::bot;
      else if (!y_)
        curFA_Pos = Robot::fA_Pos::top;
    }*/

    /*if (std::all_of(x_orien.begin(),x_orien.end(), [](bool j){return j;}))
      curFA_Pos = Robot::fA_Pos::left;
    if (std::all_of(x_orien.begin(),x_orien.end(), [](bool j){return j;}))
      curFA_Pos = Robot::fA_Pos::right;
      */

     int y_true = 0;
     int y_false = 0;
    for (i = 0; i < y_orien.size(); i++){
      if (y_orien[i] == true)
        y_true ++;
      else if (y_orien[i] == false)
        y_false ++;
    }

    if(y_false == y_orien.size()){
      curFA_Pos = Robot::fA_Pos::bot;
    }

    if(y_true == y_orien.size()){
      curFA_Pos = Robot::fA_Pos::top;
    }

     int x_true = 0;
     int x_false = 0;
    for (i = 0; i < x_orien.size(); i++){
      if (x_orien[i] == true)
        x_true ++;
      else if (x_orien[i] == false)
        x_false ++;
    }

    if(x_false == x_orien.size()){
      curFA_Pos = Robot::fA_Pos::right;
    }

    if(x_true == x_orien.size()){
      curFA_Pos = Robot::fA_Pos::left;
    }

  // For edge case
  double tHor_ =  table->GetNumber("thor", 0);
  double tVert_ = table->GetNumber("tvert", 0);

  if(curFA_Pos == Robot::fA_Pos::bot || curFA_Pos == Robot::fA_Pos::top){
    if (abs(tHor_ - tVert_) < 5)
      curFA_Pos = Robot::fA_Pos::left;

  }

  }
  frc::SmartDashboard::PutNumber("current FA Pos", curFA_Pos);

  if (m_controller2.GetBackButton())
    tarGamePiece = GamePiece::cone;
  else if (m_controller2.GetStartButton())
    tarGamePiece = GamePiece::cube;

  hasGamePiece = false;// update to ultrasnd
  table -> PutNumber("pipeline", hasGamePiece ? 0 : tarGamePiece);

  // Send Field2d to SmartDashboard.
  frc::Pose2d offPose = frc::Pose2d(frc::Translation2d(units::meter_t(-7.99),units::meter_t(-4.105)), frc::Rotation2d(units::degree_t(0)));

  int dPadAng = m_controller1.GetPOV();

  if (dPadAng>75&&dPadAng<105) {
    tarGrid = Grid::humanRight;
  } else if (dPadAng>165&&dPadAng<195) {
    tarGrid = Grid::humanCenter;
  } else if (dPadAng>255&&dPadAng<285) {
    tarGrid = Grid::humanLeft;
  }
  frc::SmartDashboard::PutNumber("Grid",tarGrid);

  isBlue = (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue);

  if (m_controller1.GetAButton()||m_controller1.GetBButton()||m_controller1.GetXButton()) {
    if (m_controller1.GetAButtonPressed()||m_controller1.GetBButtonPressed()||m_controller1.GetXButtonPressed()) {
     int lmr = 0;
     if (m_controller1.GetAButton())
      lmr = 1;
     else if (m_controller1.GetBButton())
      lmr = 2;

     int slot = 3 * tarGrid + lmr;
      /*m_swerve.setTrajCon();
      // select color
      trajectory_ = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      std::vector<frc::Pose2d> {m_swerve.GetPose(), redPose[1]},
      // Pass the config
      m_swerve.auto_traj);*/

      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
    pathplanner::PathPlannerTrajectory trajectoryPP_ = pathplanner::PathPlanner::generatePath(
    pathplanner::PathConstraints(m_swerve.kMaxSpeed, m_swerve.kMaxAcceleration), 
    pathplanner::PathPoint(m_swerve.GetPose().Translation(),m_swerve.GetPose().Rotation(), frc::Rotation2d(0_deg)), // position, heading(direction of travel), holonomic rotation
    pathplanner::PathPoint(isBlue ? bluePose[slot].Translation(): redPose[slot].Translation(),isBlue ? bluePose[slot].Rotation(): redPose[slot].Rotation(), frc::Rotation2d(0_deg) // position, heading(direction of travel) holonomic rotation
    ));

    trajectory_ = trajectoryPP_.asWPILibTrajectory();
    // Send our generated trajectory to Field2d.
    field_off.GetObject("traj")->SetTrajectory(trajectory_.RelativeTo(offPose));

      m_timer.Reset();
      // Start the timer.
      m_timer.Start();
      
    }
    if (m_timer.Get() < trajectory_.TotalTime()) {
    // Get the desired pose from the trajectory.
      auto desiredState = trajectory_.Sample(m_timer.Get());
    // Get the reference chassis speeds from the Ramsete Controller.
    
      auto refChassisSpeeds = m_holonmicController.Calculate(m_swerve.GetPose(), desiredState,frc::Rotation2d(0_deg));
        //m_ramseteController.Calculate(m_swerve.GetPose(), desiredPose);
     
 
    // Set the linear and angular speeds.
      m_swerve.Drive(refChassisSpeeds.vx, refChassisSpeeds.vy, refChassisSpeeds.omega,false);
      } 
      else {
      m_swerve.Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(0), false); 
      }
  } else if (m_controller1.GetYButton()) {
      double tx;
      if (validTarFnd) {
        tx = table -> GetNumber("tx", 0.0);
        tx *= .05;
      }
  m_swerve.DriveWithJoystick(m_controller1.GetLeftY(), 0, validTarFnd ? tx : 0, false, m_controller1.GetLeftBumper() ? true : false);
  } else {
      // Drive w joystick 0 with 50% speed if left bumper is pressed
      m_swerve.DriveWithJoystick(m_controller1.GetLeftY(),m_controller1.GetLeftX(),m_controller1.GetRightX(),true, m_controller1.GetLeftBumper() ? true : false);
  }
  if (hasGamePiece) {
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
      }
      // ----------------------------------------------------------------------------------------
  } else {
    // ----------- Update robot pose and send it to field object on DS ----------------------------- 
      // Update robot position on Field2d.
        m_swerve.UpdateOdometry();
        m_field.SetRobotPose(m_swerve.GetPose());
  }

  
  //frc::SmartDashboard::PutData(&m_field);
  
  field_off.SetRobotPose(m_field.GetRobotPose().RelativeTo(offPose));
  frc::SmartDashboard::PutData(&field_off);

  // Claw
  if (m_controller2.GetAButtonPressed()) {
    m_appendage.backRollerIn();
  } else if (m_controller2.GetYButtonPressed()) {
    m_appendage.backRollerOut();  
  } else {
    m_appendage.backRollerOff();
  }

  if (m_controller2.GetBButtonPressed()) {
    m_appendage.frontRollerIn();
  } else if (m_controller2.GetXButtonPressed()) {
    m_appendage.frontRollerOut();  
  } else {
    m_appendage.frontRollerOff();
  }

  // Arm
  m_appendage.arm(m_controller2.GetLeftY());
  
  // Shoulder
  m_appendage.shoulder(m_controller2.GetRightY());
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
