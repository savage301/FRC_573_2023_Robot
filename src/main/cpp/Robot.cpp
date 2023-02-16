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

  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  botPose = table->GetDoubleArrayTopic("botpose").Subscribe({});
  validTarget = table->GetIntegerTopic("tv").Subscribe({});
  cornerXy = table->GetDoubleArrayTopic("tcornxy").Subscribe({});

  table->PutNumber("ledMode", 0);
  table->PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//

  table->PutNumber("pipeline", 0);
  //m_swerve.ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); //Comment back in if this works see if it breaks it.
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
 
}

void Robot::AutonomousPeriodic() {
 
}

void Robot::TeleopInit() {
  // Initalize variable used in teleop.
  hasGamePiece = false;
  isBlue = (frc::DriverStation::GetAlliance() ==
            frc::DriverStation::Alliance::kBlue);  // Get Driverstation color
  tarGrid = Grid::humanLeft;
  curFA_Pos_Latch = 0;
  m_swerve.resetGyro();
  m_swerve.ResetOdometry(redPose[1]);
}

void Robot::TeleopPeriodic() {

  bool validTarFnd = validTarget.Get() > 0;
  if (validTarFnd && tarGamePiece == 1 && ((hasGamePiece ? 0 : tarGamePiece)==1)) { // Added so this doesn't run when it sees a April tag or Cube
    // ------------- Cone Orientation Code --------------------------
    // This should get moved to its own function
    if (curFA_Pos_Latch == 0) {
      std::vector<double> coneCornerXy = cornerXy.Get();
      std::vector<bool> x_orien, y_orien;
      int length = coneCornerXy.size();
      frc::SmartDashboard::PutNumber("Corner Arr Length", length);
      std::vector<double> avg = {0.0, 0.0};
      int i;
      int idxFA = 0;  // furtherest away point index
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
      uint y_true = 0;
      uint y_false = 0;
      uint j;
      for (j = 0; j < y_orien.size();
           j++) {  // Determine if the furthest point is above or below all
                   // other points in Y direction in cornerXy
        if (y_orien[j] == true)
          y_true++;
        else if (y_orien[j] == false)
          y_false++;
      }

      if (y_false == y_orien.size()) {  // If furthest point is always lt 0 then
                                        // it is on the bottom
        curFA_Pos = Robot::fA_Pos::bot;
      }

      if (y_true == y_orien.size()) {  // If furthest point is always gt 0 then
                                       // it is on the top
        curFA_Pos = Robot::fA_Pos::top;
      }

      uint x_true = 0;
      uint x_false = 0;
      for (j = 0; j < x_orien.size();
           j++) {  // Determine if the furthest point is to the left or the
                   // right of all other points in X direction in cornerXy
        if (x_orien[j] == true)
          x_true++;
        else if (x_orien[j] == false)
          x_false++;
      }

      if (x_false == x_orien.size()) {  // If furthest point is always lt 0 then
                                        // it is on the right
        curFA_Pos = Robot::fA_Pos::right;
      }

      if (x_true == x_orien.size()) {  // If furthest point is always gt 0 then
                                       // it is on the left
        curFA_Pos = Robot::fA_Pos::left;
      }

      // For edge case.
      // This sort of works. When rotating to the left form this state there is
      // a bit of an issue when just starting to be able to see the tip of the
      // cone.
      double tHor_ = table->GetNumber("thor", 0);
      double tVert_ = table->GetNumber("tvert", 0);

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
  // hasGamePiece = m_appendage.isGamePieceInClaw();
  hasGamePiece = true;
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
    double tx;
    if (tarGamePiece == Robot::GamePiece::cube) {  // Cube tracking code
      if (validTarFnd) {
        tx = table->GetNumber("tx", 0.0);
        tx *= .05;
      }
      m_swerve.DriveWithJoystick(m_controller1.GetLeftY(), 0,
                                 validTarFnd ? tx : 0, false,
                                 m_controller1.GetLeftBumper() ? true : false);

    } else if (tarGamePiece ==
               Robot::GamePiece::cone) {  // Cone Tracking section
      if (curFA_Pos == Robot::fA_Pos::bot ||
          curFA_Pos == Robot::fA_Pos::top) {  // Cone upright or tip facing
                                              // robot correctly
        tx = table->GetNumber("tx", 0.0);
        tx *= .05;
        m_swerve.DriveWithJoystick(
            m_controller1.GetLeftY(), 0, validTarFnd ? tx : 0, false,
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
              m_controller1.GetLeftY(), 0, validTarFnd ? tx : 0, false,
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
              0,
              leftRight ? -1 * m_controller1.GetLeftY()
                        : 1 * m_controller1.GetLeftY(),
              validTarFnd ? tx : 0, false,
              m_controller1.GetLeftBumper() ? true : false);
        }
      }
    }
  } /* else if (m_controller1.GetBackButton()) {
     if (m_controller1.GetBackButtonPressed()) {
       m_swerve.onRamp = false;
     }
     m_swerve.autoBalance();
   }*/
  else {
    // Default joystick driving. This is done if no other buttons are pressed on
    // driver controller
    m_swerve.DriveWithJoystick(m_controller1.GetLeftY(),
                               m_controller1.GetLeftX(),
                               m_controller1.GetRightX(), true,
                               m_controller1.GetLeftBumper() ? true : false);
  }
  // ---- End Drive Code -----------------------------------------------

  // ---- Robot Pose Generation Code -----------------------------------
  EstimatePose();
  // ---------------- End Robot Pose Generation Code -----------------


}  // End of Teleop Periodic

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

pathplanner::PathPlannerTrajectory Robot::pathGenerate(int slot) {
  // Simple path with holonomic rotation. Stationary start/end. Max velocity of
  // 4 m/s and max accel of 3 m/s^2
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
  trajectoryPP_ = pathplanner::PathPlanner::generatePath(
      pathplanner::PathConstraints(m_swerve.kMaxSpeed,
                                   m_swerve.kMaxAcceleration),
      pathplanner::PathPoint(
          m_swerve.GetPose().Translation(), m_swerve.GetPose().Rotation(),
          frc::Rotation2d(0_deg)),  // position, heading(direction of
                                    // travel),holonomic rotation
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
  //frc::SmartDashboard::PutData(&field_off);

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
      //double dTh = poseDiff.dtheta();
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

void Robot::EstimatePose(int camera_pipline) {
  if (camera_pipline==0) {
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
      //double dTh = poseDiff.dtheta();
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
