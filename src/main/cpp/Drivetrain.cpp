// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>

/*
 * Remaps a number
 */
double Drivetrain::remapVal(double i, double threshold) {
  if (std::abs(i) > threshold) {
    i = i / std::abs(i) * threshold;
  }

  return i;
}

double Drivetrain::deadband(double i, double threshold) {
  if (std::abs(i) < threshold) {
    i = 0;
  }

  return i;
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  frc::SwerveModulePosition FL = {m_frontLeft.GetPosition().distance,
                                  m_frontLeft.GetPosition().angle};
  frc::SwerveModulePosition FR = {m_frontRight.GetPosition().distance,
                                  m_frontRight.GetPosition().angle};
  frc::SwerveModulePosition BL = {m_backLeft.GetPosition().distance,
                                  m_backLeft.GetPosition().angle};
  frc::SwerveModulePosition BR = {m_backLeft.GetPosition().distance,
                                  m_backRight.GetPosition().angle};

  m_poseEstimator.Update(m_gyro.GetRotation2d(), {FL, FR, BL, BR});

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  // m_poseEstimator.AddVisionMeasurement(
  //    ExampleGlobalMeasurementSensor::GetEstimatedGlobalPose(
  //        m_poseEstimator.GetEstimatedPosition()),
  //    frc::Timer::GetFPGATimestamp() - 0.3_s);
}

void Drivetrain::UpdateOdometry(frc::Pose2d camerapose, double latency) {

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  m_poseEstimator.AddVisionMeasurement(camerapose,
                                       frc::Timer::GetFPGATimestamp() - units::second_t(latency));

  frc::SwerveModulePosition FL = {m_frontLeft.GetPosition().distance,
                                  m_frontLeft.GetPosition().angle};
  frc::SwerveModulePosition FR = {m_frontRight.GetPosition().distance,
                                  m_frontRight.GetPosition().angle};
  frc::SwerveModulePosition BL = {m_backLeft.GetPosition().distance,
                                  m_backLeft.GetPosition().angle};
  frc::SwerveModulePosition BR = {m_backLeft.GetPosition().distance,
                                  m_backRight.GetPosition().angle};

  m_poseEstimator.Update(m_gyro.GetRotation2d(), {FL, FR, BL, BR});

  
}

frc::ChassisSpeeds Drivetrain::GetRobotVelocity() {
  return m_kinematics.ToChassisSpeeds(
      {m_frontLeft.GetState(), m_frontRight.GetState(), m_backLeft.GetState(),
       m_backRight.GetState()});
}

void Drivetrain::DriveWithJoystick(double xJoy, double yJoy, double rJoy,
                                   bool fieldRelative, bool lim,
                                   bool gyrostablize) {
  // Weikai: add bool lim to limit speed to 50%

  frc::SmartDashboard::PutNumber("xJoy", xJoy);
  frc::SmartDashboard::PutNumber("yJoy", yJoy);
  frc::SmartDashboard::PutNumber("rJoy", rJoy);

  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed =
      Drivetrain::m_xspeedLimiter.Calculate(frc::ApplyDeadband(xJoy, 0.02)) *
      kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed =
      Drivetrain::m_yspeedLimiter.Calculate(frc::ApplyDeadband(yJoy, 0.02)) *
      kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.

  if (gyrostablize) {
    rJoy = gryoStablize();
    frc::SmartDashboard::PutNumber("gryoStabrot", rJoy);
  } else {
    gyroSetpoint = m_gyro.GetAngle();
  }

  const auto rot =
      Drivetrain::m_rotLimiter.Calculate(frc::ApplyDeadband(rJoy, 0.02)) *
      kMaxAngularSpeed;

  Drive(lim ? xSpeed / 2 : xSpeed, lim ? ySpeed / 2 : ySpeed,
        lim ? rot / 3 : rot, fieldRelative);
  // UpdateOdometry();
  frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_poseEstimator.ResetPosition(
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}

frc::Pose2d Drivetrain::GetPose() const {
  return m_poseEstimator.GetEstimatedPosition();
}

void Drivetrain::setTrajCon() {
  auto_traj.SetKinematics(m_kinematics);
}

void Drivetrain::autoBalance(bool mobility) {
  double vector = .99;
  frc::SmartDashboard::PutNumber("RampState", rampState);
  vector = m_gyro.GetRoll();
  double coeff = vector / std::abs(vector); // need to confirm roll positive  vs negative value this may need negative symbol
  switch (rampState) {
    case 0: {
      DriveWithJoystick(-fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      if (std::abs(vector) > balancedZ){
        if (mobility)
          rampState+=4;
        else{
          rampState++;
          counter = 0;
        }
      }
      break;
    }
    case 1: {
      /*double kP = 0.003, val = 0;
      if (std::abs(vector) > balancedZ) {
          val = std::abs(vector) * kP;
      }*/
      if (counter < 7){
DriveWithJoystick(coeff * midSpeed, zeroSpeed, zeroSpeedrot, true, false,
                        true);
         counter ++;
      }
      else{
      DriveWithJoystick(coeff * midSpeed, zeroSpeed, zeroSpeedrot, true, false,
                        true);
      if (std::abs(vector) < RampZ) {
        counter = 0;
        rampState++;
      }
      }
      break;
    }
    case 2: {
      if (counter < 25) {
        DriveWithJoystick(zeroSpeed, zeroSpeed, zeroSpeedrot, true,
                          false, true);
        counter++;
      } else {
        if (std::abs(vector) < balancedZ)
        stopDrivetrain(false, 0);
       else {
        rampState++;
        counter = 0;
       }

      }
      break;
    }
    case 3: {
      if (std::abs(vector) > balancedZ) {
        DriveWithJoystick(coeff * slowestSpeed, zeroSpeed, zeroSpeedrot, true,
                          false, true);
        counter++;
      } else {
        stopDrivetrain(false, 0);
      }
      break;
    }
    case 4: {
      DriveWithJoystick(-fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      if (std::abs(vector) < balancedZ){
          rampState++;
      }
      break;
    }
    case 5: {
      DriveWithJoystick(-fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      if (std::abs(vector) > balancedZ){
          rampState++;
          counter = 0;
      }
      break;
    }
    case 6: { // on floor back side case
      DriveWithJoystick(-fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      if (std::abs(vector) < balancedZ){
          if (counter > 5)
            rampState++;
          else
            counter++;
      }
      break;
    }
    case 7: { // on floor back side case
      DriveWithJoystick(fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      counter = 0;
      if (std::abs(vector) > balancedZ){
          rampState = 1;
          }
      break;
    }

    default: {
      if (counter < 5) {
        stopDrivetrain(false, 0.025);
        counter++;
      } else {
        stopDrivetrain(true, 0);
      }
    }
  }
  

  // -----Cross Ramp Section ---------------------------

  /*if(currRampPos == Drivetrain::RampPos::floor && std::abs(gV[2]) > balancedZ
  && !crossedramp){
    DriveWithJoystick(-fastSpeed,zeroSpeed,zeroSpeedrot,true,false);
      currRampPos = Drivetrain::RampPos::floor;
  }
  else if (currRampPos == Drivetrain::RampPos::floor && std::abs(gV[2]) < RampZ
  && !crossedramp){ currRampPos = Drivetrain::RampPos::upward; lastRampSide =
  Drivetrain::RampPos::upward;
    DriveWithJoystick(-fastSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }

  else if (currRampPos == Drivetrain::RampPos::upward && std::abs(gV[2]) >
  balancedZ && !crossedramp){ currRampPos = Drivetrain::RampPos::balanced;
    DriveWithJoystick(-fastSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }

  else if (currRampPos == Drivetrain::RampPos::balanced && std::abs(gV[2]) <
  RampZ && !crossedramp){ currRampPos = Drivetrain::RampPos::downside;
    lastRampSide = Drivetrain::RampPos::downside;
    DriveWithJoystick(-fastSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }

  else if (currRampPos == Drivetrain::RampPos::downside && std::abs(gV[2]) >
  balancedZ && !crossedramp){ currRampPos = Drivetrain::RampPos::floorback;
    DriveWithJoystick(-fastSpeed,zeroSpeed,zeroSpeedrot,true,false);
    crossedramp = true;
  }

  //
  --------------------------------------------------------------------------------
  // - Balance Section --------------------------------------------
  else if (currRampPos == Drivetrain::RampPos::floorback && std::abs(gV[2]) <
  RampZ && crossedramp){ currRampPos = Drivetrain::RampPos::downside;
    lastRampSide = Drivetrain::RampPos::downside;
    DriveWithJoystick(fastSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }

  else if ((currRampPos == Drivetrain::RampPos::downside || currRampPos ==
  Drivetrain::RampPos::upward) && std::abs(gV[2]) > balancedZ && crossedramp){
    currRampPos = Drivetrain::RampPos::balanced;
    DriveWithJoystick(zeroSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }

  else if (currRampPos == Drivetrain::RampPos::balanced && std::abs(gV[2]) <
  RampZ && crossedramp && lastRampSide == Drivetrain::RampPos::downside){
    currRampPos = Drivetrain::RampPos::upward;
    lastRampSide = Drivetrain::RampPos::upward;
    DriveWithJoystick(-slowSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }
   else if (currRampPos == Drivetrain::RampPos::balanced && std::abs(gV[2]) <
  RampZ && crossedramp && lastRampSide == Drivetrain::RampPos::upward){
    currRampPos = Drivetrain::RampPos::downside;
    lastRampSide = Drivetrain::RampPos::downside;
    DriveWithJoystick(slowSpeed,zeroSpeed,zeroSpeedrot,true,false);
  }*/
}

/*void Drivetrain::autoBalanceWithMobility() {
  double vector = .99;
  frc::SmartDashboard::PutNumber("RampState", rampState);
  vector = m_gyro.GetRoll();
  frc::SmartDashboard::PutNumber("Vector", vector);
  double coeff = vector / std::abs(vector);
  switch (rampState) {
    case RampPos::floor: {
      DriveWithJoystick(-fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      if (std::abs(vector) < balancedZ) {
        rampState += 4;
        counter = 0;
      }
      break;
    }
    case RampPos::floorback: {
      DriveWithJoystick(fastSpeed, zeroSpeed, zeroSpeedrot, true, false, true);
      if (std::abs(vector) > balancedZ)
        rampState++;

      break;
    }

    case 1: {
      DriveWithJoystick(-coeff * midSpeed, zeroSpeed, zeroSpeedrot, true, false,
                        true);
      if (std::abs(vector) < balancedZ) {
        counter = 0;
        rampState++;
      }
      break;
    }
    case 2: {
      if (counter < 5) {
        DriveWithJoystick(-coeff * slowestSpeed, zeroSpeed, zeroSpeedrot, true,
                          false, false);
        counter++;
      } else {
        stopDrivetrain(true, 0);
      }
      if (std::abs(vector) > RampZ)
        rampState--;
      break;
    }
    default: {
      if (counter < 5) {
        stopDrivetrain(false, 0.025);
        counter++;
      } else {
        stopDrivetrain(true, 0);
      }
    }
  }
}*/

void Drivetrain::pumpOutSensorVal() {
  pumpOutNum("Gyro angle", m_gyro.GetAngle());
  pumpOutNum("Gyro Roll", m_gyro.GetRoll());
}

bool Drivetrain::isGyroWorking() {
  double cur;
  cur = m_gyro.GetAngle();
  if (cur != last) {  // current is not equal to previous
    last = cur;
    return true;
  }
  last = cur;
  return false;
}

void Drivetrain::resetGyro(double angle) {
  m_gyro.SetYaw(angle, 0);
}

void Drivetrain::updateGyroAngle() {
  gyroSetpoint = m_gyro.GetAngle();
}

double Drivetrain::gryoStablize() {
  return gyro_PIDController.Calculate(m_gyro.GetAngle(), gyroSetpoint);
}

void Drivetrain::resetDrivetrain() {
  m_frontLeft.resetTurningMotorHeading();
  m_frontRight.resetTurningMotorHeading();
  m_backLeft.resetTurningMotorHeading();
  m_backRight.resetTurningMotorHeading();
  resetGyro(0);
}

void Drivetrain::stopDrivetrain(bool gyro, double r) {
  if (gyro)
    r = gryoStablize();
  else
    updateGyroAngle();

  const auto rot =
      Drivetrain::m_rotLimiter.Calculate(frc::ApplyDeadband(r, 0.02)) *
      kMaxAngularSpeed;
  Drive(units::velocity::meters_per_second_t{0},
        units::velocity::meters_per_second_t{0}, rot, false);
}

void Drivetrain::updateMotorIdleMode(bool auton) {
  m_frontLeft.switchIdleMode(auton);
  m_frontRight.switchIdleMode(auton);
  m_backLeft.switchIdleMode(auton);
  m_backRight.switchIdleMode(auton);
}
