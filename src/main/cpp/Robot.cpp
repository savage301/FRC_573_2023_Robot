// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define frcLog frc::DataLogManager::DataLogManager
wpi::log::StringLogEntry m_log;

#define addToChooser(x) m_chooser.AddOption(x, x);

void Robot::RobotInit() {
  m_appendage.pneumaticsOut();
  m_appendage.frontClawPneumaticsIn();
  m_appendage.backClawPneumaticsIn();
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  addToChooser(kAutoNameCustom);
  addToChooser(kAutonPaths1);
  addToChooser(kAutonPaths2);
  addToChooser(kAutonPaths3);
  addToChooser(kAutonPaths4);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  botPose = table->GetDoubleArrayTopic("botpose").Subscribe({});
  validTarget = table->GetIntegerTopic("tv").Subscribe({});
  cornerXy = table->GetDoubleArrayTopic("tcornxy").Subscribe({});

  table->PutNumber("ledMode", 0);
  table->PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//

  table->PutNumber("pipeline", 0);

  frcLog::Start();

  frc::DriverStation::StartDataLog(frcLog::GetLog());

  compressor.EnableDigital();
  m_swerve.ResetOdometry(
      frc::Pose2d{5_m, 5_m, 0_deg});  // flipping the robot for field setup
  frc::SmartDashboard::PutBoolean("Ignore appendage limits",
                                  m_appendage.unleashThePower);
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
  // reset pos based on selector
  if (m_autoSelected == kAutonPaths1)
    // start with 6 / 2
    m_swerve.ResetOdometry(redPose[6]);
  else if (m_autoSelected == kAutonPaths2)
    m_swerve.ResetOdometry(redPose[2]);
  else if (m_autoSelected == kAutonPaths3)
    m_swerve.ResetOdometry(bluePose[6]);
  else if (m_autoSelected == kAutonPaths4)
    m_swerve.ResetOdometry(bluePose[2]);

  else if (m_autoSelected == kAutonPaths5)
    m_swerve.ResetOdometry(redPose[3]);
  else if (m_autoSelected == kAutonPaths6)
    m_swerve.ResetOdometry(bluePose[3]);
  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected =
      frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  // fmt::print("Auto selected: {}\n", m_autoSelected);

  autoState = 0;
  isBlue = (frc::DriverStation::GetAlliance() ==
            frc::DriverStation::Alliance::kBlue);  // Get Driverstation color
  m_appendage.appendageReset(false);

  // ---------------------------------- Trajectory Following Auto Section
  // --------------------- Generate trajectory to follow for autonomous Start
  // the timer.
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
  if (m_autoSelected == kAutonPaths1)  // update init pose?
    autonomousPaths(1);
  else if (m_autoSelected == kAutonPaths2)
    autonomousPaths(2);
  else if (m_autoSelected == kAutonPaths3)
    autonomousPaths(3);
  else if (m_autoSelected == kAutonPaths4)
    autonomousPaths(4);

  else if (m_autoSelected == kAutonPaths5)
    driveToCS(false);
  else if (m_autoSelected == kAutonPaths6)
    driveToCS(true);

  frc::SmartDashboard::PutNumber(
      "m_timer",
      m_timer.Get()
          .value());  // This will allow us to debug the auto drive code.
}

void Robot::TeleopInit() {
  // Initalize variable used in teleop.
  hasGamePiece = false;
  isBlue = (frc::DriverStation::GetAlliance() ==
            frc::DriverStation::Alliance::kBlue);  // Get Driverstation color
  tarGrid = Grid::humanLeft;
  tarGamePiece = Robot::GamePiece::cone;
  curFA_Pos_Latch = 0;
  m_swerve.resetGyro();
  m_swerve.ResetOdometry(redPose[1]);
}

void Robot::TeleopPeriodic() {
  // pump out sensor values to dashboard for diagnostics
  m_appendage.pumpOutSensorVal();
  m_swerve.pumpOutSensorVal();
  // getPowerDistribution();
  frc::SmartDashboard::PutNumber("Has Game Piece", hasGamePiece);
  frc::SmartDashboard::PutNumber("Target Game Piece", tarGamePiece);
  frc::SmartDashboard::PutNumber(
      "m_timer",
      m_timer.Get()
          .value());  // This will allow us to debug the auto drive code.
  if (m_controller1.GetLeftBumper())
    m_appendage.frontClawPneumaticsOut();
  else
    m_appendage.frontClawPneumaticsIn();

  if (m_controller1.GetRightBumper())
    m_appendage.backClawPneumaticsOut();
  else
    m_appendage.backClawPneumaticsIn();

  bool validTarFnd = validTarget.Get() > 0;

  if (validTarFnd) {
    // ------------- Cone Orientation Code --------------------------
    // This should get moved to its own function
    if (curFA_Pos_Latch == 0) {
      std::vector<double> coneCornerXy = cornerXy.Get();
      std::vector<bool> x_orien, y_orien;
      std::vector<double> avg = {0.0, 0.0};
      int i, idxFA = 0,
             /* furtherest away point index*/ length = coneCornerXy.size();
      frc::SmartDashboard::PutNumber("Corner Arr Length", length);
      double FADist = 0;
      for (i = 0; i < length;
           i += 2)  // Calculate avg point location from cornerXy
      {
        avg[0] += coneCornerXy[i];
        avg[1] += coneCornerXy[i + 1];
      }
      avg[0] /= (.5 * i);
      avg[1] /= (.5 * i);
      frc::SmartDashboard::PutNumber("avg x", avg[0]);
      frc::SmartDashboard::PutNumber("avg y", avg[1]);
      for (i = 0; i < length;
           i += 2)  // Determine point in cornerXy furthest from the average
                    // point location in cornerXy
      {
        double tmpDistFA =
            std::sqrt(std::pow((coneCornerXy[i] - avg[0]), 2) +
                      std::pow((coneCornerXy[i + 1] - avg[1]), 2));
        if (tmpDistFA >= FADist) {
          FADist = tmpDistFA;
          idxFA = i;
        }
      }
      frc::SmartDashboard::PutNumber("index of FA", idxFA);
      for (i = 0; i < length;
           i += 2)  // Determine if furthest away point is + or - from all other
                    // points in x and y in cornerXy
      {
        double x, y;
        if (i != idxFA) {
          x = coneCornerXy[i] - coneCornerXy[idxFA];
          y = coneCornerXy[i + 1] - coneCornerXy[idxFA + 1];

          x_orien.push_back(x > 0);
          y_orien.push_back(y > 0);
        }
      }
      uint y_true = 0, y_false = 0, j;
      for (j = 0; j < y_orien.size();
           j++) {  // Determine if the furthest point is above or below all
                   // other points in Y direction in cornerXy
        if (y_orien[j] == true)
          y_true++;
        else if (y_orien[j] == false)
          y_false++;
      }

      if (y_false == y_orien.size())  // If furthest point is always lt 0 then
                                      // it is on the bottom
        curFA_Pos = Robot::fA_Pos::bot;

      if (y_true == y_orien.size())  // If furthest point is always gt 0 then
                                     // it is on the top
        curFA_Pos = Robot::fA_Pos::top;

      uint x_true = 0, x_false = 0;
      for (j = 0; j < x_orien.size();
           j++) {  // Determine if the furthest point is to the left or the
                   // right of all other points in X direction in cornerXy
        if (x_orien[j] == true)
          x_true++;
        else if (x_orien[j] == false)
          x_false++;
      }

      if (x_false == x_orien.size())  // If furthest point is always lt 0 then
                                      // it is on the right
        curFA_Pos = Robot::fA_Pos::right;

      if (x_true == x_orien.size())  // If furthest point is always gt 0 then
                                     // it is on the left
        curFA_Pos = Robot::fA_Pos::left;

      // For edge case.
      // This sort of works. When rotating to the left form this state there is
      // a bit of an issue when just starting to be able to see the tip of the
      // cone.
      double tHor_ = table->GetNumber("thor", 0),
             tVert_ = table->GetNumber("tvert", 0);

      if (curFA_Pos == Robot::fA_Pos::bot || curFA_Pos == Robot::fA_Pos::top) {
        if (std::abs(tHor_ - tVert_) <
            5) {  // If measured horz and vert sides are within 5px we assume
                  // its a square is only happens if the cone is facing the
                  // wrong way
          curFA_Pos =
              Robot::fA_Pos::left;  // Set to left to handle this edge case
          curFA_Pos_Latch =
              50;  // Sets a 10 loop latch to reduce jitter in this usecase
        }
      }
    } else {
      curFA_Pos_Latch--;
    }

    // ------------- End Cone Orientation Code --------------------------
  }
  frc::SmartDashboard::PutNumber("current FA Pos", curFA_Pos);

  selectGamePiece();
  /*if (m_appendage.getAnalogWorking()) // Not expected for 1st event
    hasGamePiece = m_appendage.isGamePieceInClaw();
  else
    updateHasGamePiece();*/
  updateHasGamePiece();
  table->PutNumber(
      "pipeline",
      hasGamePiece ? 0 : tarGamePiece);  // Sets limelight pipeline (0 for April
                                         // Tag, 1 for cone, 2 for cube)

  // ------------------ Drive Code --------------------------------------
  selectScoringGrid();
  if (m_controller1.GetAButton() || m_controller1.GetBButton() ||
      m_controller1.GetXButton()) {
    // This section is for path following code to each scoring location.
    if (m_controller1.GetAButtonPressed() ||
        m_controller1.GetBButtonPressed() ||
        m_controller1.GetXButtonPressed()) {
      // This if statement is only run first time through loop, or once each
      // time the button is pressed.
      int lmr = 0;
      if (m_controller1.GetAButton())
        lmr = 1;
      else if (m_controller1.GetBButton())
        lmr = 2;

      int slot = 3 * tarGrid + lmr;
      driveWithTraj(pathGenerate(slot), offPose);
    }
    driveWithTraj(false);
  } else if (m_controller1.GetYButton()) {
    if (m_controller1.GetYButtonPressed()) {
      curFA_Pos_Latch = 0;
    }
    // This section is for the auto game piece tracking code.
    double tx, tmp = std::sqrt(std::pow(m_controller1.GetLeftX(), 2) +
                               std::pow(m_controller1.GetLeftY(), 2));
    if (tarGamePiece == Robot::GamePiece::cube) {  // Cube tracking code
      if (validTarFnd) {
        tx = table->GetNumber("tx", 0.0);
        tx *= .05;
      }
      m_swerve.DriveWithJoystick(tmp, 0, validTarFnd ? tx : 0, false,
                                 m_controller1.GetLeftBumper() ? true : false);

    } else if (tarGamePiece ==
               Robot::GamePiece::cone) {  // Cone Tracking section
      if (curFA_Pos == Robot::fA_Pos::bot ||
          curFA_Pos == Robot::fA_Pos::top) {  // Cone upright or tip facing
                                              // robot correctly
        tx = table->GetNumber("tx", 0.0);
        tx *= .05;
        m_swerve.DriveWithJoystick(
            tmp, 0, validTarFnd ? tx : 0, false,
            m_controller1.GetLeftBumper() ? true : false);
      } else if (curFA_Pos == Robot::fA_Pos::left ||
                 curFA_Pos == Robot::fA_Pos::right) {  // Cone tipped over, but
                                                       // no oriented correctly.
        // This section drives towards the tipped cone until it is a certian
        // size in the camera then rotates around the cone.
        double ta = table->GetNumber("ta", 0.0);
        if (ta < 1) {  // Approach the cone until it is a certain size in image.
          tx = table->GetNumber("tx", 0.0);
          tx *= .05;
          m_swerve.DriveWithJoystick(
              tmp, 0, validTarFnd ? tx : 0, false,
              m_controller1.GetLeftBumper() ? true : false);
        } else {  // Rotate around the cone either left or right.
          bool leftRight = false;
          tx = table->GetNumber("tx", 0.0);
          tx *= .05;
          if (curFA_Pos == Robot::fA_Pos::left)
            leftRight = false;
          else if (curFA_Pos == Robot::fA_Pos::right)
            leftRight = true;
          m_swerve.DriveWithJoystick(
              0, leftRight ? -1 * tmp : 1 * tmp, validTarFnd ? tx : 0, false,
              m_controller1.GetLeftBumper() ? true : false);
        }
      }
    }
    /* else if (m_controller1.GetBackButton()) {
       if (m_controller1.GetBackButtonPressed()) {
         m_swerve.onRamp = false;
       }
       m_swerve.autoBalance();
     }*/
  } else {
    // Default joystick driving. This is done if no other buttons are pressed on
    // driver controller
    m_swerve.DriveWithJoystick(
        m_controller1.GetLeftY(), m_controller1.GetLeftX(),
        m_controller1.GetRightX(), true,
        m_controller1.GetLeftBumper() || m_appendage.getArmExtended() ? true
                                                                      : false);
  }
  // ---- End Drive Code -----------------------------------------------

  // ---- Robot Pose Generation Code -----------------------------------
  EstimatePose();
  // ---------------- End Robot Pose Generation Code -----------------

  // ---------------- Appendage Code ---------------------------------
  // Claw
  if (tarGamePiece == Robot::GamePiece::cube) {
    m_appendage.pneumaticsIn();
    if (m_controller2.GetLeftTriggerAxis() > .5) {
      m_appendage.frontRollerIn();
      m_appendage.backRollerIn();
    } else if (m_controller2.GetRightTriggerAxis() > .5) {
      m_appendage.frontRollerOut();
      m_appendage.backRollerOut();
    } else {
      m_appendage.frontRollerOff();
      m_appendage.backRollerOff();
    }
  }

  if (tarGamePiece == Robot::GamePiece::cone) {
    if (m_controller2.GetLeftTriggerAxis() > .5) {
      m_appendage.frontRollerIn();
      m_appendage.backRollerIn();
      m_appendage.pneumaticsOut();
    } else if (m_controller2.GetRightTriggerAxis() > .5) {
      m_appendage.frontRollerOut();
      m_appendage.backRollerOut();
      m_appendage.pneumaticsOut();
    } else if (m_controller2.GetLeftBumper()) {
      m_appendage.backRollerIn();
      m_appendage.frontRollerOff();
      m_appendage.pneumaticsIn();
    } else {
      m_appendage.frontRollerOff();
      m_appendage.backRollerOff();
      m_appendage.pneumaticsOut();
    }
  }

// --------- All possible Arm Positions ---------------------
  if (m_controller2.GetAButton() && hasGamePiece) {
    // Floor Level Drop Off
    if (m_controller2.GetRightBumper()) 
      m_appendage.armPID(0);
     else
      m_appendage.armPID(0);
   
    m_appendage.shoulderPID(-1994);  
    
    if (m_controller2.GetLeftBumper()||tarGamePiece==Robot::GamePiece::cube)   
      m_appendage.wristPID(2100);
     else 
      m_appendage.wristPID(1326);
    
  } else if (m_controller2.GetBButton() && hasGamePiece) {
    // Mid Level Scoring Location
    if (m_controller2.GetRightBumper()) 
      m_appendage.armPID(0);
    else 
      m_appendage.armPID(0);
    
    if (tarGamePiece == Robot::GamePiece::cone)
      m_appendage.shoulderPID(-1049);  
    else
      m_appendage.shoulderPID(-881);  

    m_appendage.wristPID(2100);

  } else if (m_controller2.GetYButton() && hasGamePiece) {
    // Upper Level Scoring
    if (tarGamePiece == Robot::GamePiece::cone) {
      if (m_controller2.GetRightBumper()) 
        m_appendage.armPID(-168);
      else 
        m_appendage.armPID(0);
      
      m_appendage.shoulderPID(-716);  

    } else {
      if (m_controller2.GetRightBumper()) 
        m_appendage.armPID(-131);
      else 
        m_appendage.armPID(0);
      
      m_appendage.shoulderPID(-683);  
    }

    m_appendage.wristPID(2100);
  } 
  
  else if (m_controller2.GetAButton() && !hasGamePiece) {
    // Floor Level Pickup
    if (m_controller2.GetRightBumper()) 
      m_appendage.armPID(0);
     else
      m_appendage.armPID(0);
   
    m_appendage.shoulderPID(-1994);  
    
    if (m_controller2.GetLeftBumper()||tarGamePiece==Robot::GamePiece::cube)   
      m_appendage.wristPID(2050);
     else 
      m_appendage.wristPID(1539);
    
  } else if (m_controller2.GetBButton() && !hasGamePiece) {
    // Side Human Player Loading Location
    if (m_controller2.GetRightBumper()) 
      m_appendage.armPID(0);
    else 
      m_appendage.armPID(0);
    
    if (tarGamePiece == Robot::GamePiece::cone)
      m_appendage.shoulderPID(-1049);  
    else
      m_appendage.shoulderPID(-881);  

    m_appendage.wristPID(2100);

  } else if (m_controller2.GetYButton() && !hasGamePiece) {
    // End field Human Player Loading Location
    if (tarGamePiece == Robot::GamePiece::cone) {
      if (m_controller2.GetRightBumper()) 
        m_appendage.armPID(-188);
      else 
        m_appendage.armPID(0);
      
      m_appendage.shoulderPID(-716);  

    } else {
      if (m_controller2.GetRightBumper()) 
        m_appendage.armPID(-131);
      else 
        m_appendage.armPID(0);
      
      m_appendage.shoulderPID(-683);  
    }

    m_appendage.wristPID(2100);
  }  
  else if (m_controller2.GetXButton()) {
    // Store Position possible from with or without gamepiece
    m_appendage.armPID(0);
    m_appendage.shoulderPID(0);  // Stored Position
    m_appendage.wristPID(0);
  } else {
    // Arm
    // if going up and is closer to the lim
    if (m_appendage.calculateDistanceToLim() <= 2 &&
        (m_controller2.GetLeftY() < 0))
      m_appendage.arm(0);
    else
      m_appendage.arm(m_controller2.GetLeftY());

    // Shoulder
    // if going up and is closer to the lim
    if (m_appendage.calculateDistanceToLim() <= 2 &&
        (m_controller2.GetRightY() < 0))
      m_appendage.shoulder(0);
    else
      m_appendage.shoulder(m_controller2.GetRightY());

    m_appendage.wrist(m_controller2.GetLeftX());
  }

  // --------- End All possible Arm Positions ---------------------
  // ----------- End Appendage Code -----------------------------------
  handleLedModes(validTarFnd, hasGamePiece, tarGamePiece,
                 m_appendage.checkEdge());
}  // End of Teleop Periodic

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void frc::BangBangController::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("BangBangController");
  /*builder.AddDoubleProperty(
      "tolerance", [this] { return GetTolerance(); },
      [this](double tolerance) { SetTolerance(tolerance); });
  builder.AddDoubleProperty(
      "setpoint", [this] { return GetSetpoint(); },
      [this](double setpoint) { SetSetpoint(setpoint); });
  builder.AddDoubleProperty(
      "measurement", [this] { return GetMeasurement(); }, nullptr);
  builder.AddDoubleProperty(
      "error", [this] { return GetError(); }, nullptr);
  builder.AddBooleanProperty(
      "atSetpoint", [this] { return AtSetpoint(); }, nullptr);
      */
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

pathplanner::PathPlannerTrajectory Robot::pathGenerate(int slot) {
  // Simple path with holonomic rotation. Stationary start/end. Max velocity of
  // 4 m/s and max accel of 3 m/s^2
  auto robotvelo =
      units::meters_per_second_t(
          std::pow(std::pow(m_swerve.GetRobotVelocity().vx.value(), 2) +
                       std::pow(m_swerve.GetRobotVelocity().vy.value(), 2),
                   0.5))
          .value();
  frc::SmartDashboard::PutNumber("Robot Vel", robotvelo);

  trajectoryPP_ = pathplanner::PathPlanner::generatePath(
      pathplanner::PathConstraints(m_swerve.kMaxSpeed,
                                   m_swerve.kMaxAcceleration),
      pathplanner::PathPoint(
          m_swerve.GetPose().Translation(), m_swerve.GetPose().Rotation(),
          frc::Rotation2d(0_deg)),  // position, heading(direction of travel),
                                    // holonomic rotation, optional velocity in
                                    // the current heading of travel in mps
      pathplanner::PathPoint(
          isBlue ? bluePose[slot].Translation() : redPose[slot].Translation(),
          isBlue ? bluePose[slot].Rotation() : redPose[slot].Rotation(),
          frc::Rotation2d(0_deg)  // position, heading(direction of travel)
                                  // holonomic rotation, optional velocity in
                                  // the current heading of travel in mps
          ));
  return trajectoryPP_;
}

pathplanner::PathPlannerTrajectory Robot::pathGenerate(frc::Pose2d tarPose) {
  // Simple path with holonomic rotation. Stationary start/end. Max velocity of
  // 4 m/s and max accel of 3 m/s^2

  auto robotvelo =
      units::meters_per_second_t(
          std::pow(std::pow(m_swerve.GetRobotVelocity().vx.value(), 2) +
                       std::pow(m_swerve.GetRobotVelocity().vy.value(), 2),
                   0.5))
          .value();

  frc::SmartDashboard::PutNumber("Robot Vel", robotvelo);
  trajectoryPP_ = pathplanner::PathPlanner::generatePath(
      pathplanner::PathConstraints(m_swerve.kMaxSpeed,
                                   m_swerve.kMaxAcceleration),
      pathplanner::PathPoint(
          m_swerve.GetPose().Translation(), m_swerve.GetPose().Rotation(),
          frc::Rotation2d(
              0_deg)),  // position, heading(direction of
                        // travel),holonomic rotation, robot velocity
      pathplanner::PathPoint(
          tarPose.Translation(), tarPose.Rotation(),
          frc::Rotation2d(0_deg)  // position, heading(direction of travel)//
                                  // holonomic rotation
          ));
  return trajectoryPP_;
}

void Robot::driveWithTraj(pathplanner::PathPlannerTrajectory trajectoryPP_,
                          frc::Pose2d offPose) {
  trajectory_ = trajectoryPP_.asWPILibTrajectory();

  // Send our generated trajectory to Dashboard Field Object
  field_off.GetObject("traj")->SetTrajectory(trajectory_.RelativeTo(offPose));
  // frc::SmartDashboard::PutData(&field_off);

  // Start the timer for trajectory following.
  m_timer.Reset();
  m_timer.Start();
}

void Robot::driveWithTraj(bool auton) {
  if (m_timer.Get() < trajectory_.TotalTime()) {
    auto desiredState = trajectory_.Sample(
        m_timer.Get());  // Get the desired pose from the trajectory.

    auto refChassisSpeeds = m_holonmicController.Calculate(
        m_swerve.GetPose(), desiredState, frc::Rotation2d(0_deg));

    frc::SmartDashboard::PutNumber("pose x", m_swerve.GetPose().X().value());
    frc::SmartDashboard::PutNumber("desired pose x",
                                   desiredState.pose.X().value());

    frc::SmartDashboard::PutNumber("pose y", m_swerve.GetPose().Y().value());
    frc::SmartDashboard::PutNumber("desired pose y",
                                   desiredState.pose.Y().value());
    // Set the linear and angular speeds.
    m_swerve.Drive(refChassisSpeeds.vx, refChassisSpeeds.vy,
                   refChassisSpeeds.omega, false);
  } else {
    // When trajectory is completed if button is still pressed this stops the
    // robot
    m_swerve.Drive(units::meters_per_second_t(0), units::meters_per_second_t(0),
                   units::radians_per_second_t(0), false);
    if (auton) {
      autoState++;
      m_timer.Reset();
      firstTime = true;
    }
  }
}

#define setLeds(x) m_leds.led_control(x)
void Robot::handleLedModes(bool isGamePiece, bool isGamePieceAcquired,
                           int tarGamePiece, bool isEdgeClose) {
  if (isEdgeClose) {
    if (isGamePieceAcquired) {
      setLeds("Green");
      if (isGamePiece) {
        setLeds("White");
        if (tarGamePiece == Robot::GamePiece::cone)
          setLeds("Yellow");
        else if (tarGamePiece == Robot::GamePiece::cube)
          setLeds("Purple");
      }
    }
  } else {
    setLeds("Black");
  }
}

#define pumpOut frc::SmartDashboard::PutNumber
void Robot::getPowerDistribution() {
  frc::PowerDistribution bd =
      frc::PowerDistribution(20, frc::PowerDistribution::ModuleType::kRev);
  pumpOut("intake motor 1 current", bd.GetCurrent(4));
  pumpOut("intake motor 2 current", bd.GetCurrent(5));
  pumpOut("arm motor current", bd.GetCurrent(0));
  pumpOut("shoulder motor current", bd.GetCurrent(12));
  pumpOut("wrist motor current", bd.GetCurrent(3));

  pumpOut("drive motor BR 1", bd.GetCurrent(11));
  pumpOut("drive motor BR 2", bd.GetCurrent(10));
  pumpOut("drive motor BL 1", bd.GetCurrent(7));
  pumpOut("drive motor BL 2", bd.GetCurrent(8));
  pumpOut("drive motor FR 1", bd.GetCurrent(5));
  pumpOut("drive motor FR 2", bd.GetCurrent(4));
  pumpOut("drive motor FL 1", bd.GetCurrent(1));
  pumpOut("drive motor FL 2", bd.GetCurrent(2));
}

void Robot::selectGamePiece() {
  // Set target piece status variable
  if (m_controller2.GetBackButton())
    tarGamePiece = GamePiece::cone;
  else if (m_controller2.GetStartButton())
    tarGamePiece = GamePiece::cube;
}

void Robot::selectScoringGrid() {
  // Target Grid selection code for auto path following
  int dPadAng = m_controller1.GetPOV();
  if (dPadAng > 75 && dPadAng < 105) {
    tarGrid = Grid::humanRight;
  } else if (dPadAng > 165 && dPadAng < 195) {
    tarGrid = Grid::humanCenter;
  } else if (dPadAng > 255 && dPadAng < 285) {
    tarGrid = Grid::humanLeft;
  }
  frc::SmartDashboard::PutNumber("Grid", tarGrid);
}

void Robot::autonomousPaths(bool isBlue, int slot, frc::Pose2d poseMidPoint,
                            frc::Pose2d poseCube) {
  switch (autoState) {
    case 0: {
      table->PutNumber("pipeline", 0);  // April Tag Camera Pipeline
      m_swerve.DriveWithJoystick(0, 0, 0, false, false);
      EstimatePose(0);
      bool wristReady = m_appendage.wristPID(1);
      bool armReady = m_appendage.armPID(1);
      bool shoulderReady = m_appendage.shoulderPID(1);

      if (wristReady && armReady && shoulderReady) {
        m_timer.Reset();
        m_timer.Start();
        autoState++;
      }

      break;
    }
    case 1: {
      m_appendage.appendageReset(true);
      EstimatePose(0);
      if (m_timer.Get().value() > .25) {
        m_timer.Stop();
        autoState++;
        firstTime = true;
      }
      break;
    }
    case 2: {
      if (firstTime) {
        trajectoryPP_ = pathGenerate(poseMidPoint);  // mid pt
        driveWithTraj(trajectoryPP_, offPose);
      }
      firstTime = false;
      m_appendage.armPID(-1);
      driveWithTraj(true);
      EstimatePose(0);

      break;
    }
    case 3: {
      table->PutNumber("pipeline", 2);  // Cube Pipeline
      if (firstTime) {
        trajectoryPP_ = pathGenerate(poseCube);
        driveWithTraj(trajectoryPP_, offPose);
      }
      firstTime = false;
      m_appendage.armPID(0);
      m_appendage.shoulderPID(-1);
      m_appendage.frontRollerIn();
      m_appendage.backRollerIn();
      m_appendage.pneumaticsIn();
      driveWithTraj(true);
      EstimatePose(2);
      break;
    }
    case 4: {
      table->PutNumber("pipeline", 2);  // Cube Pipeline
      double tx;
      bool validTarFnd = validTarget.Get() > 0;
      if (validTarFnd) {
        tx = table->GetNumber("tx", 0.0);
        tx *= .05;
      }
      m_swerve.DriveWithJoystick(-.8, 0, validTarFnd ? tx : 0, false, false);
      EstimatePose(2);
      if (m_appendage.isGamePieceInClaw() || m_timer.Get().value() > 2 ||
          isPassCenterLine()) {
        autoState++;
        m_timer.Reset();
        firstTime = true;
      }
      break;
    }
    case 5: {
      table->PutNumber("pipeline", 0);  // April Tag Pipeline
      if (firstTime) {
        trajectoryPP_ = pathGenerate(poseMidPoint);  // mid pt
        driveWithTraj(trajectoryPP_, offPose);
      }
      firstTime = false;
      m_appendage.armPID(0);
      m_appendage.shoulderPID(1);
      m_appendage.frontRollerOff();
      m_appendage.backRollerOff();
      m_appendage.pneumaticsIn();
      driveWithTraj(true);
      EstimatePose(0);
      break;
    }
    case 6: {
      if (firstTime) {
        trajectoryPP_ = pathGenerate(slot);
        driveWithTraj(trajectoryPP_, offPose);
      }
      firstTime = false;
      m_appendage.arm(0);
      m_appendage.shoulderPID(1);
      driveWithTraj(true);
      EstimatePose(0);

      break;
    }
    case 7: {
      m_swerve.DriveWithJoystick(0, 0, 0, false, false);
      EstimatePose(0);
      if (m_appendage.armPID(1)) {
        autoState++;
        m_timer.Reset();
        firstTime = true;
      }
      break;
    }
    case 8: {
      m_appendage.backRollerOut();
      m_appendage.frontRollerOut();
      m_swerve.DriveWithJoystick(0, 0, 0, false, false);
      EstimatePose(0);
      if (m_timer.Get().value() > .5) {
        m_timer.Reset();
        m_timer.Start();
        autoState++;
      }

      break;
    }
    default: {
      m_swerve.DriveWithJoystick(0, 0, 0, false, false);
      EstimatePose(0);
      m_appendage.armPID(1);
      m_appendage.backRollerOff();
      m_appendage.frontRollerOff();
      break;
    }
  }
}
void Robot::autonomousPaths(int select) {
  switch (select) {
    case 1: {  // red right
      autonomousPaths(false, 7, redRightMidPose, redRightcube);
      break;
    }
    case 2: {  // red left
      autonomousPaths(false, 1, redLeftMidPose, redLeftcube);
      break;
    }
    case 3: {  // blue right
      autonomousPaths(false, 7, blueRightMidPose, blueRightcube);
      break;
    }
    case 4: {  // blue left
      autonomousPaths(false, 1, blueLeftMidPose, blueLeftcube);
      break;
    }
    default:
      break;
  }
}

bool Robot::isPassCenterLine() {
  frc::Pose2d curPose = m_swerve.GetPose();
  double curY = curPose.Y().value();
  if (std::abs(curY) < .305)
    return true;
  return false;
}

void Robot::EstimatePose() {
  if (hasGamePiece) {
    // If robot has game piece use April tags to attempt to localize robot
    std::vector<double> robotPose = botPose.Get();
    bool validTarFnd = validTarget.Get() > 0;
    if (validTarFnd && robotPose.size() > 0) {
      // If robot can see atleast 1 april tag and has a returned botPose from
      // the limelight it localizes base on botPose
      frc::SmartDashboard::PutNumber("robotPoseX", robotPose[0]);
      frc::SmartDashboard::PutNumber("robotPoseY", robotPose[1]);
      frc::SmartDashboard::PutNumber("robotPoseYaw", robotPose[5]);
      frc::Translation2d tmp2d = frc::Translation2d(
          units::meter_t(robotPose[0]), units::meter_t(robotPose[1]));
      frc::Rotation2d tmpAng = frc::Rotation2d(units::degree_t(robotPose[5]));
      frc::Pose2d fldPose = frc::Pose2d(tmp2d, tmpAng);

      frc::Twist2d poseDiff = m_swerve.GetPose().Log(fldPose);
      double dx = poseDiff.dx();
      double dy = poseDiff.dy();
      // double dTh = poseDiff.dtheta();
      double r = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      if (r < 1) {
        m_swerve.UpdateOdometry(fldPose);
      } else {
        m_swerve.UpdateOdometry();
      }
    } else {
      // No April tags can be seen the robot updates Pose based on wheel
      // odometry.
      m_swerve.UpdateOdometry();
    }
  } else {
    // When robot doesn't have a game piece robot updates Pose based on wheel
    // odometry, because camera will be using wrong pipeline
    m_swerve.UpdateOdometry();
  }

  // Send pose data to DS
  m_field.SetRobotPose(m_swerve.GetPose());
  field_off.SetRobotPose(m_field.GetRobotPose().RelativeTo(offPose));
  frc::SmartDashboard::PutData(&field_off);

  frc::ChassisSpeeds robotSpeed = m_swerve.GetRobotVelocity();

  frc::SmartDashboard::PutNumber("Robot Speed X", robotSpeed.vx.value());
  frc::SmartDashboard::PutNumber("Robot Speed Y", robotSpeed.vy.value());
  frc::SmartDashboard::PutNumber("Robot Speed Omega", robotSpeed.omega.value());
}

void Robot::EstimatePose(int camera_pipline) {
  if (camera_pipline == 0) {
    // If robot has game piece use April tags to attempt to localize robot
    std::vector<double> robotPose = botPose.Get();
    bool validTarFnd = validTarget.Get() > 0;
    if (validTarFnd && robotPose.size() > 0) {
      // If robot can see atleast 1 april tag and has a returned botPose from
      // the limelight it localizes base on botPose
      frc::SmartDashboard::PutNumber("robotPoseX", robotPose[0]);
      frc::SmartDashboard::PutNumber("robotPoseY", robotPose[1]);
      frc::SmartDashboard::PutNumber("robotPoseYaw", robotPose[5]);
      frc::Translation2d tmp2d = frc::Translation2d(
          units::meter_t(robotPose[0]), units::meter_t(robotPose[1]));
      frc::Rotation2d tmpAng = frc::Rotation2d(units::degree_t(robotPose[5]));
      frc::Pose2d fldPose = frc::Pose2d(tmp2d, tmpAng);

      frc::Twist2d poseDiff = m_swerve.GetPose().Log(fldPose);
      double dx = poseDiff.dx();
      double dy = poseDiff.dy();
      // double dTh = poseDiff.dtheta();
      double r = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

      if (r < 1) {
        m_swerve.ResetOdometry(fldPose);
      } else {
        m_swerve.UpdateOdometry();
      }
    } else {
      // No April tags can be seen the robot updates Pose based on wheel
      // odometry.
      m_swerve.UpdateOdometry();
    }
  } else {
    // When robot doesn't have a game piece robot updates Pose based on wheel
    // odometry, because camera will be using wrong pipeline
    m_swerve.UpdateOdometry();
  }

  // Send pose data to DS
  m_field.SetRobotPose(m_swerve.GetPose());
  field_off.SetRobotPose(m_field.GetRobotPose().RelativeTo(offPose));
  frc::SmartDashboard::PutData(&field_off);
}

double Robot::estimateGamePieceDistanceToCenter() {
  double claw1 = 0, claw2 = 0;
  if (m_appendage.isGamePieceInClaw()) {
    // calc rel to center
    claw1 = std::abs(m_appendage.getClaw1());
    claw2 = std::abs(m_appendage.getClaw2());
  }
  // if (claw1 > 80 && claw2 > 80)
  //  do this

  // else if (claw2)
  return claw1 - claw2;
}

void Robot::driveToCS(bool isBlue) {
  switch (autoState) {
    case 0: {
      table->PutNumber("pipeline", 0);  // April Tag Camera Pipeline
      m_swerve.DriveWithJoystick(0, 0, 0, false, false);
      EstimatePose(0);
      bool wristReady = m_appendage.wristPID(1);
      bool armReady = m_appendage.armPID(1);
      bool shoulderReady = m_appendage.shoulderPID(1);

      if (wristReady && armReady && shoulderReady) {
        m_timer.Reset();
        m_timer.Start();
        autoState++;
      }

      break;
    }
    case 1: {
      m_appendage.appendageReset(true);
      EstimatePose(0);
      if (m_timer.Get().value() > .25) {
        m_timer.Stop();
        autoState++;
        firstTime = true;
      }
      break;
    }
    case 2: {
      if (firstTime) {
        trajectoryPP_ =
            pathGenerate(isBlue ? blueCharge : redCharge);  // mid of the cs
        driveWithTraj(trajectoryPP_, offPose);
      }
      firstTime = false;
      m_appendage.armPID(-1);
      driveWithTraj(true);
      EstimatePose(0);
      m_appendage.backClawPneumaticsOut();
      break;
    }
    case 3:
      m_swerve.autoBalance();
      break;
    default: {
      m_swerve.DriveWithJoystick(0, 0, 0, false, false);
      EstimatePose(0);
      m_appendage.armPID(1);
      m_appendage.backRollerOff();
      m_appendage.frontRollerOff();
      break;
    }
  }
}

void Robot::updateHasGamePiece() {
  int dPadAng = m_controller2.GetPOV();
  if (dPadAng > 75 && dPadAng < 105)
    hasGamePiece = true;
  else if (dPadAng > 255 && dPadAng < 285)
    hasGamePiece = false;
}
