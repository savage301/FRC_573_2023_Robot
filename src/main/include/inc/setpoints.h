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

const double wrist_max = 745;
const double wrist_min = 0;

// home
const double armHome = 10;
const double shoulderHome = 0;
const double wristHome = 10;

// floor pickup
// const double armFloor = ;
const double shoulderFloor = -2100;
const double wristFloorCubeLoad = wristHome; // 250
const double wristFloorConeLoad = wristHome; // 175
const double wristFloor = wristHome; // 465

// floor score
const double wristFloorCubeScore = wristHome;
const double wristFloorConeScore = wristHome;

// mid score
const double armMidCube = armHome;
const double shoulderMidCube = -700; 
const double shoulderMidCone = shoulderMidCube; // -515
const double wristMidCone = wristHome; //490
const double wristMidCube = wristHome; //350

// high score
const double armHighCube = -112;
const double armHighCone = -110;
const double shoulderHighCube = -800;
const double shoulderHighCone = shoulderHighCube; // -500
const double wristHigh = 490;
const double wristHighCube = wristHome; //350
const double wristHighCone = wristHome;       //490

// high human player
const double armHumanHigh = armHome;
const double shoulderHumanHigh = 535;
const double wristHumanHigh = wristHome; // 730

// low human player
const double shoulderHumanLow = 1400;
const double wristHumanLow = wristHome;
