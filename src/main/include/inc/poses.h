// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <vector>

#define pose1(x, y) frc::Pose2d(x, y, frc::Rotation2d(0_deg))
#define pose_(x, y, o) frc::Pose2d(x, y, frc::Rotation2d(o))
#define pose2(x, y) frc::Pose2d(x, y, frc::Rotation2d(180_deg))
#define poseCubes(x, y) \
  frc::Pose2d(x, y, frc::Rotation2d(180_deg))  // facing away from the wall
#define poseRed(y) pose1(6.41_m, y)
#define poseBlue(y) pose2(-6.41_m, y)
#define poseRedMid(y) \
  pose1(5.09_m, y)  // not sure 'bout this, might be 180, i.e. pose2
#define poseBlueMid(y) pose1(-5.09_m, y)
#define poseRedCubes_(y) poseCubes(1.2_m, y)
#define poseBlueCubes_(y) poseCubes(-1.2_m, y)

frc::Pose2d poseTestStart = pose_(6.41_m, -1.5_m, -180_deg);
frc::Pose2d poseTestEnd = pose_(3.4_m, -1.5_m, -180_deg);
frc::Pose2d botTestStart = pose_(6.41_m, -1.5_m, 0_deg);

std::vector<frc::Pose2d> redPose = {   // slot num
    poseRed(-3.5_m),                   // 0
    poseRed(-2.94_m),                  // 1
    poseRed(-2.38_m),                  // 2
    poseRed(-1.82_m),                  // 3
    poseRed(-1.26_m),                  // 4
    poseRed(-.7_m),                    // 5
    poseRed(-.14_m),                   // 6
    poseRed(.42_m),                    // 7
    poseRed(.98_m)};                   // 8
std::vector<frc::Pose2d> bluePose = {  // slot num
    poseBlue(.98_m),                   // 0
    poseBlue(.42_m),                   // 1
    poseBlue(-.14_m),                  // 2
    poseBlue(-.7_m),                   // 3
    poseBlue(-1.26_m),                 // 4
    poseBlue(-1.82_m),                 // 5
    poseBlue(-2.38_m),                 // 6
    poseBlue(-2.94_m),                 // 7
    poseBlue(-3.5_m)};                 // 8

frc::Pose2d redRightMidPose = poseRedMid(0.67_m);
frc::Pose2d redLeftMidPose = poseRedMid(-3.17_m);
frc::Pose2d redRightcube = poseRedCubes_(.57_m);
frc::Pose2d redLeftcube = poseRedCubes_(-3.09_m);

frc::Pose2d blueLeftMidPose = poseBlueMid(0.67_m);
frc::Pose2d blueRightMidPose = poseBlueMid(-3.17_m);
frc::Pose2d blueLeftcube = poseBlueCubes_(.57_m);
frc::Pose2d blueRightcube = poseBlueCubes_(-3.09_m);

frc::Pose2d redCharge = pose1(5.88_m, -1.29_m);
frc::Pose2d blueCharge = pose1(-5.88_m, -1.29_m);
