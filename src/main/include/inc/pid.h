// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include "physics.h"

// Appendage
static frc2::PIDController Arm_PIDController{0.1, 0, 0};
static frc2::PIDController Shoulder_PIDController{-0.005, 0, 0};
static frc2::PIDController Wrist_PIDController{-0.005, 0, 0};

// Drivetrain
static frc2::PIDController gyro_PIDController{-0.0075, 0, 0};
