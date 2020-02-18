#pragma once

#include <units/units.h>

#include "frc/RobotController.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogEncoder.h"

#include "PenguinUtil.h"
#include <wpi/math>

#include "rev/CANEncoder.h"

struct TurnEncoder {
 private:
  int port;
  units::radian_t offset;
  frc::AnalogInput encoderAsAnalogInput;
  frc::AnalogEncoder encoderAsAnalogEncoder;
  rev::CANEncoder builtInMotorEncoder;
  frc::Rotation2d offsetRot;

 public:
  TurnEncoder(int port, units::radian_t offset, rev::CANEncoder sparkMaxEncoder) : 
    port{port},
    offset{offset},
    encoderAsAnalogInput{port},
    encoderAsAnalogEncoder{encoderAsAnalogInput},
    builtInMotorEncoder{sparkMaxEncoder},
    offsetRot{offset} {
      #define INVERT_BUILTIN_ENCODER_I_THINK 1 // or -1
      builtInMotorEncoder.SetPositionConversionFactor(INVERT_BUILTIN_ENCODER_I_THINK * PenguinUtil::TWO_PI / (18. / 1.)); // 18:1 is the default angle reduction
      builtInMotorEncoder.SetPosition(GetAngle().to<double>());
  }


  /** The current angle of the module according to the analog encoder.
   * 
   * Business logic is copied from SDS` `angleSupplier()` (by way of `getAngle()` and `readAngle()`): https://github.com/FRCTeam2910/Common/blob/a240f39c9c3d3ae9f1e74c11cffe07e314c743bd/robot/src/main/java/org/frcteam2910/common/robot/drivers/Mk2SwerveModuleBuilder.java#L90
   * The analog encoder is continuous, but this code norms the angle to between -pi and pi.
   * @param continuous when true, returns the actual value of the encoder instead of the equivalent terminal angle between -pi and pi. Defaults to false.
   * @return the module angle, as a units::radian_t
   */
  units::radian_t GetAngle(bool continuous = false) const {
    units::radian_t angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * PenguinUtil::TWO_PI_RAD;
    angle += offset;
    // angle *= -1;

    if (!continuous) {
      // double angle_d = fmod(angle.to<double>(), PenguinUtil::TWO_PI);
      // angle = units::radian_t(angle_d);
      angle = units::math::fmod(angle, PenguinUtil::TWO_PI_RAD);

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

  units::radian_t GetMotorEncoderPosition() {
    return units::radian_t(builtInMotorEncoder.GetPosition());
  }

  /** Probably don't use; not maintaining feature parity with `GetAngle`.
   * Attempts (perhaps successfully?) to use the auto-terminal-angle features of Rotation2d instead of computing the modulus.
   * @return the current angle of the module
   */
  units::radian_t GetAngle2() const {
    double angle = (1.0 - encoderAsAnalogInput.GetVoltage() / frc::RobotController::GetVoltage5V()) * (2.0 * wpi::math::pi);
    frc::Rotation2d ret = frc::Rotation2d(units::radian_t(angle));
    ret += offsetRot;
    return ret.Radians();
  }

  void UpdateOffset() {
    this->offset += GetAngle();
  }
};