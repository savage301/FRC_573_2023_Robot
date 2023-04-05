// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Appendage Encoder Limits
const double arm_max = -5;
const double arm_min = -125.0;

const double shoulder_max = 1900;
const double shoulder_min = -2400.0;

/*
const double wrist_max = 333;
const double wrist_min = 290;
*/
const double wrist_max = 0;
const double wrist_min = -7.5;

// home
const double armHome = 10;
const double shoulderHome = 0;
const double wristHome = -0.1;

// floor pickup
// const double armFloor = ;
const double shoulderFloor = -2375;
const double wristFloorCubeLoad = -2.6;
const double wristFloorConeLoad = -1.8;
const double wristFloor = -3;

// floor score
const double wristFloorCubeScore = wristHome;
const double wristFloorConeScore = wristHome;

// mid score
const double armMidCube = armHome;
const double shoulderMidCube = -800; 
const double shoulderMidCone = -730; // -515
const double wristMidCone = -4.4;
const double wristMidCube = -4.4; //350

// high score
const double armHighCube = -112;
const double armHighCone = -115;
const double shoulderHighCube = -700;
const double shoulderHighCone = -600; // -500
const double wristHigh = 490;
const double wristHighCube = -5; //350
//const double wristHighCone = 305;       //490
const double wristHighCone = -4.7;       //490
const double shoulderHighCubeAuto = -600;

// high human player
const double armHumanHigh = armHome;
const double shoulderHumanHigh = 730;
const double wristHumanHigh = -6.8; // 730

// low human player
const double shoulderHumanLow = 1750;
const double wristHumanLow = wristHome;
