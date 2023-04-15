// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "def.h"

// Swerve
static constexpr double kWheelRadius = 0.047625;
static constexpr int kDriveGearRatio = 6.75;
static constexpr int kSteerEncoderResolution = 4096;

static constexpr auto kModuleMaxAngularVelocity =
    Pi * 10_rad_per_s;  // radians per second
static constexpr auto kModuleMaxAngularAcceleration =
    Pi * 20_rad_per_s_sq;  // radians per second^2

static constexpr units::meters_per_second_t kMaxSpeed =
    4.4_mps;  // 14.5 ft/s to meters per second

static constexpr units::radians_per_second_t kMaxAngularSpeed{
    Pi * 2};  // rotation per second

static constexpr auto kMaxAcceleration = 2.2_mps_sq;  // meters per second^2

static constexpr units::radians_per_second_squared_t kMaxAngularAccel{
    Pi};  // 1 rotation per second per second

static constexpr frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{
    1_V, 3_V / 1_mps};
static constexpr frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
    1_V, 0.5_V / 1_rad_per_s};

// Auto balance
const double RampZ = 10;
const double balancedZ = 7;  // 5 works for dock
const double fastSpeed = 0.5;
const double midSpeed = 0.1;
// double slowSpeed = .045;
const double slowestSpeed = 0.021;
const double zeroSpeed = 0;
const double zeroSpeedrot = 0;

// Appendage
constexpr double clawWidth = 0.045;  // 17.875 inches / 2 in meters
