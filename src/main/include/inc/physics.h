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

// Swerve
static constexpr double kWheelRadius = 0.047625;
static constexpr int kDriveGearRatio = 6.75;
static constexpr int kSteerEncoderResolution = 4096;

static constexpr auto kModuleMaxAngularVelocity =
    std::numbers::pi * 10_rad_per_s;  // radians per second
static constexpr auto kModuleMaxAngularAcceleration =
    std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2

static constexpr units::meters_per_second_t kMaxSpeed =
    4.4_mps;  // 14.5 ft/s to meters per second

static constexpr units::radians_per_second_t kMaxAngularSpeed{
    std::numbers::pi * 2};  // rotation per second

// Acceleration = (4.4 m/s)^2 / (2 * 0.047625 m * 6.75) = 5.91 m/s^2
static constexpr auto kMaxAcceleration =
    units::meters_per_second_squared_t(5.9);  // meters per second^2

static constexpr units::radians_per_second_squared_t kMaxAngularAccel{
    std::numbers::pi};  // 1 rotation per second per second

static constexpr frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{
    1_V, 3_V / 1_mps};
static constexpr frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
    1_V, 0.5_V / 1_rad_per_s};
