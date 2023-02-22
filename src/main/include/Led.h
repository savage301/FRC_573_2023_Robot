// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "frc/motorcontrol/Spark.h"

class Led {
 private:
  // Define motor, sensors, and pnematic pointers here
  frc::Spark* m_leds;

 public:
  Led();
  // Define Led class functions here
  void led_control(std::string input);
};
