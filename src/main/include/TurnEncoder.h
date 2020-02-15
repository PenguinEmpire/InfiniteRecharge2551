#pragma once

#include <units/units.h>

#include "frc/RobotController.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogEncoder.h"

#include "PenguinUtil.h"
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
      builtInMotorEncoder.SetPositionConversionFactor(PenguinUtil::TWO_PI / (18. / 1.)); // 18:1 is the default angle reduction
  }

  units::radian_t GetAngle_SDS() const {
    units::radian_t angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * PenguinUtil::TWO_PI_RAD;
    angle += offset;

    double angle_d = fmod(angle.to<double>(), PenguinUtil::TWO_PI);

    angle = units::radian_t(angle_d);

    while (angle > PenguinUtil::PI_RAD) {
      angle -= PenguinUtil::TWO_PI_RAD;
    }
    while (angle < -PenguinUtil::PI_RAD) {
      angle += PenguinUtil::TWO_PI_RAD;
    }
    // if (angle < 0_rad) { // TODO: why is this commented out?
    //   angle += 360_deg;
    // }
    
    return angle;
  }

  units::radian_t GetAngle2() const {
    double angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * (2.0 * wpi::math::pi);
    frc::Rotation2d ret = frc::Rotation2d(units::radian_t(angle));
    ret += offsetRot;
    return ret.Radians();
  }

  void UpdateOffset() {
    offset += GetAngle_SDS();
  }
};