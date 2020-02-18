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

  units::radian_t currentAngle;

  TurnEncoder(int port, units::radian_t offset, rev::CANEncoder sparkMaxEncoder) : 
    port{port},
    offset{offset},
    encoderAsAnalogInput{port},
    encoderAsAnalogEncoder{encoderAsAnalogInput},
    builtInMotorEncoder{sparkMaxEncoder},
    offsetRot{offset} {
#define INVERT_BUILTIN_ENCODER_I_THINK 1
      builtInMotorEncoder.SetPositionConversionFactor(INVERT_BUILTIN_ENCODER_I_THINK * PenguinUtil::TWO_PI / (18. / 1.)); // 18:1 is the default angle reduction
      builtInMotorEncoder.SetPosition(GetAngle_SDS().to<double>());
  }

  units::radian_t GetAngle_SDS(bool continuous = false) const {
    units::radian_t angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * PenguinUtil::TWO_PI_RAD;
    angle += offset;
    // angle *= -1;

    if (!continuous) {
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
    }
      
    return angle;
  }

  /** Probably don't use; not maintaining feature parity with `GetAngle_SDS`.
   * Attempt to use the auto-terminal-angle features of Rotation2d instead of computing the modulus.
   * @return the current angle of the module
   */
  units::radian_t GetAngle2() const {
    double angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * (2.0 * wpi::math::pi);
    frc::Rotation2d ret = frc::Rotation2d(units::radian_t(angle));
    ret += offsetRot;
    return ret.Radians();
  }

  void UpdateOffset() {
    this->offset += GetAngle_SDS();
  }
};