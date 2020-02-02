#pragma once

#include <units/units.h>

#include "frc/RobotController.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogEncoder.h"

/** For some reason, the linker complains about multiple definitions when this is included.
 * (Even though I have `#pragma once`! I even tried an if guard. No dice.)
 * It was only included so the 2pi could be a constant the couple times it's used.
 * PenguinUtil::TWO is defined as `2.0 * wpi::math::pi`, and I've replaced the three places
 * it occured with that expression.
 */
// #include "PenguinUtil.h"
#include <wpi/math>

#include "rev/CANEncoder.h"

struct TurnEncoder {
  int port;
  units::radian_t offset;
  frc::AnalogInput encoderAsAnalogInput;
  frc::AnalogEncoder encoderAsAnalogEncoder;
  rev::CANEncoder builtInMotorEncoder;
  frc::Rotation2d offsetRot;

  TurnEncoder(int port, units::radian_t offset, rev::CANEncoder sparkMaxEncoder) : 
    port{port},
    offset{offset},
    encoderAsAnalogInput{port},
    encoderAsAnalogEncoder{encoderAsAnalogInput},
    builtInMotorEncoder{sparkMaxEncoder},
    offsetRot{offset} {
      builtInMotorEncoder.SetPositionConversionFactor(2.0 * wpi::math::pi / (18. / 1.)); // 18:1 is the default angle reduction
    }

  units::radian_t GetAngle_SDS() const {
    double angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * (2.0 * wpi::math::pi);
    angle += offset.to<double>();
    angle = fmod(angle, (2.0 * wpi::math::pi));
    if (angle < 0) {
      angle += (2.0 * wpi::math::pi);
    }
    return units::radian_t(angle);
  }

  units::radian_t GetAngle2() const {
    double angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * (2.0 * wpi::math::pi);
    frc::Rotation2d ret = frc::Rotation2d(units::radian_t(angle));
    ret += offsetRot;
    return ret.Radians();
  }
};