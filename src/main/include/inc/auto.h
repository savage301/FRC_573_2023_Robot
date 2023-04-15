// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <string>
#include <vector>

// Placeholder
const std::string kAutoNameDefault = "Default";
const std::string kAutoNameCustom = "Nothing";

// Old non-working paths
const std::string kAutonPaths1 = "Red Right 6 to 7";
const std::string kAutonPaths2 = "Red Left 2 to 1";
const std::string kAutonPaths3 = "Blue Right 6 to 7";
const std::string kAutonPaths4 = "Blue Left 2 to 1";
const std::string kAutonPaths5 = "Red Left 3 to CS";
const std::string kAutonPaths6 = "Blue Left 3 to CS";

const std::string kAutonPaths45 = "Red Left 3 M to CS";
const std::string kAutonPaths46 = "Blue Left 3 M to CS";

// Basic paths
const std::string kAutonPaths7 = "Basic Red Right Right";
const std::string kAutonPaths8 = "Basic Red Left Left";
const std::string kAutonPaths9 = "Basic Blue Right Right";
const std::string kAutonPaths10 = "Basic Blue Left Left";

const std::string kAutonPaths12 = "Basic Blue Cone High No Mobility";
const std::string kAutonPaths13 = "Basic Red Cone High No Mobility";

// New test paths
const std::string kAutonPaths11 = "simple turn";
const std::string kAutonPaths98 = "2 gp";
const std::string kAutonPaths99 = "3 gp";

// Pathplanner paths
const std::string twoGPPathRed = "2 GP red";
const std::string twoGPPathBlue = "2 GP blue";
const std::string threeGPPath = "3rd";

// new separated ones
const std::string twoGPpt1 = "2GPpt1";
const std::string twoGPpt2 = "2GPpt2;";
const std::string twoGPpt3 = "2GPpt3";
const std::string twoGPpt4 = "2GPpt4";

// Basic 2 Piece Paths
const std::string kAutonPaths50 = "Basic 2 Piece Red";
const std::string kAutonPaths51 = "Basic 2 Piece Blue";
const std::string kAutonPaths52 = "Basic 2 Piece Red Cable Tray";
const std::string kAutonPaths53 = "Basic 2 Piece Blue Cable Tray";

const std::vector<std::string> autoModes = {
    /*kAutonPaths1,
    kAutonPaths2,
    kAutonPaths3,
    kAutonPaths4,
    */
    kAutonPaths5,
    kAutonPaths6,
    kAutonPaths7,
    kAutonPaths8,
    kAutonPaths9,
    kAutonPaths10,
    kAutonPaths11,

    // mo mobility
    kAutonPaths12,
    kAutonPaths13,
    // no mobility

    kAutonPaths45,
    kAutonPaths46,
    kAutonPaths50,
    kAutonPaths51,

    // cable tray
    kAutonPaths52,
    kAutonPaths53,
    //

    kAutonPaths98,
    kAutonPaths99,
};
