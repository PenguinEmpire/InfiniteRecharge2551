#pragma once

#include <units/units.h>

#include "frc/RobotController.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogEncoder.h"

#include "PenguinUtil.h"

struct TurnEncoder {
  int port;
  units::radian_t offset;
  frc::AnalogInput encoderAsAnalogInput;
  frc::AnalogEncoder encoderAsAnalogEncoder;

  TurnEncoder(int port, units::radian_t offset) : 
    port{port},
    offset{offset},
    encoderAsAnalogInput{port},
    encoderAsAnalogEncoder{encoderAsAnalogInput} {}

  units::radian_t getAngle_SDS() const {
    double angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * PenguinUtil::TWO_PI;
    angle += offset.to<double>();
    angle = fmod(angle, PenguinUtil::TWO_PI);
    if (angle < 0) {
      angle += PenguinUtil::TWO_PI;
    }
    return units::radian_t(angle);
  }
};