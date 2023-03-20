// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Appendage Encoder Limits
const double arm_max = -5;
const double arm_min = -125.0;

const double shoulder_max = 1700;
const double shoulder_min = -2000.0;

const double wrist_max = 4000.0;
const double wrist_min = 0.0;

// home
const double armHome = 10;
const double shoulderHome = 0;
const double wristHome = 100;

// floor pickup
// const double armFloor = ;
const double shoulderFloor = -1994;
const double wristFloorCubeLoad = 1900;
const double wristFloorConeLoad = 1539;
const double wristFloor = 1326;

// floor score
const double wristFloorCubeScore = wristHome;
const double wristFloorConeScore = wristHome;

// mid score
const double armMidCube = armHome;
const double shoulderMidCube = -1150;
const double shoulderMidCone = -540;
const double wristMidCone = 2200;
const double wristMidCube = wristHome;

// high score
const double armHighCube = -112;
const double armHighCone = -105;
const double shoulderHighCube = -800;
const double shoulderHighCone = -650;
const double wristHigh = 2445;
const double wristHighCube = wristHome;
const double wristHighCone = 2445;

// high human player
const double armHumanHigh = armHome;
const double shoulderHumanHigh = 535;
const double wristHumanHigh = 3700;

// low human player
const double shoulderHumanLow = 1600;
const double wristHumanLow = wristHome;