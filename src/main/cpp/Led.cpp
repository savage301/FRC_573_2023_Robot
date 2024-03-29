// Copyright (c) 2023 FIRST and other WPILib contributors.
// Copyright (c) 2023 FRC Team 573
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Led.h"

#include "frc\Joystick.h"

Led::Led() {
  // Define CAN and PWM Ids used in Led here
  int LedID = 9;

  // Define motors, sensors, and pneumatics here
  m_leds = new frc::Spark(LedID);
}

void Led::led_control(std::string input) {
  // LED control through Blinkin need to add to if else tree if adding any new
  // patterens or colors Pattern list
  // http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf

  if (input == "Orange") {
    m_leds->Set(0.62);
  } else if (input == "Blue") {
    m_leds->Set(0.87);
  } else if (input == "Hot_Pink") {
    m_leds->Set(0.57);
  } else if (input == "Heartbeat") {
    m_leds->Set(0.05);
  } else if (input == "Red") {
    m_leds->Set(0.61);
  } else if (input == "Party_Mode") {
    m_leds->Set(-0.89);
  } else if (input == "Black") {
    m_leds->Set(0.99);
  } else if (input == "White") {
    m_leds->Set(0.93);
  } else if (input == "Green") {
    m_leds->Set(0.77);
  } else if (input == "Rainbow") {
    m_leds->Set(-0.99);
  } else if (input == "Purple") {
    m_leds->Set(0.91);  // violet
  } else if (input == "Yellow") {
    m_leds->Set(0.69);
  } else {
    m_leds->Set(0.99);
  }
}
