#pragma once

#include <math.h>
#include <wpi/math>

#include <units/units.h>
#include "frc/geometry/Rotation2d.h"

namespace PenguinUtil {
  inline double linearMap(double n, double start1, double stop1, double start2, double stop2) {
    // Credit to the Processing Foundation and p5.js: https://github.com/processing/p5.js/blob/86d6b67707965526ce11cf893e26be5d53a1ad4c/src/math/calculation.js#L461
    return (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
  }

  inline double deadband(double input, double deadband = 0.05) {
    if (fabs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  inline double tooSmartDeadband(double input, double deadbandStart, double deadbandEnd, double leftRangeStart, double leftRangeEnd, double valueInDeadband, double rightRangeStart, double rightRangeEnd) {
    if (deadbandStart <= input && input <= deadbandEnd) {
      return valueInDeadband;
    } else if (-1 <= input && input < deadbandStart) {
      return linearMap(input, -1, deadbandStart, leftRangeStart, leftRangeEnd);
    } else if (deadbandEnd < input && input <= 1) {
      return linearMap(input, deadbandEnd, 1, rightRangeStart, rightRangeEnd);
    } else {
      return 0.0;
      printf("Line %i: !!! Invalid input (abs(input) > 1) being passed to `PenguinUtil::tooSmartDeadband`. Returned 0.", __LINE__);
    }
  }

  inline double smartDeadband(double input, double deadbandStart, double deadbandEnd, double valueAtDeadBandEdge = 0.1) {
    return tooSmartDeadband(input, deadbandStart, deadbandEnd, -1, -valueAtDeadBandEdge, 0, valueAtDeadBandEdge, 1);
  }

  inline bool approxEqual(double value1, double value2, double tolerance = 0.01) {
    return fabs(value1 - value2) < tolerance;
  }
 
  inline constexpr double PI = wpi::math::pi; // TODO: inline only on functions?
  inline constexpr double TWO_PI = 2.0 * wpi::math::pi;
  inline constexpr units::radian_t PI_RAD = units::radian_t(PI);
  inline constexpr units::radian_t TWO_PI_RAD = units::radian_t(TWO_PI);
  inline constexpr frc::Rotation2d ZERO_ROT = frc::Rotation2d();
  inline const frc::Rotation2d PI_ROT = frc::Rotation2d(PI_RAD);
  inline const frc::Rotation2d TWO_PI_ROT = frc::Rotation2d(TWO_PI_RAD);

  inline units::radian_t arbitraryTwoPiRangeNorm(units::radian_t value, units::radian_t mid) {
    units::radian_t ret = value;

    while (ret > mid + 180_deg) {
      ret -= 360_deg;
    }
    while (ret < mid - 180_deg) {
      ret += 360_deg;
    }

    return ret;
  }

  inline units::radian_t piNegPiNorm(units::radian_t value) {
    return arbitraryTwoPiRangeNorm(value, 0_deg);
  }
} // PenguinUtil namespace

namespace PenguinConstants {
  constexpr double LIMELIGHT_DEFAULT_VALUE = 0.0;

  namespace CAN {
    constexpr int ELEVATOR_MASTER = 9;
    constexpr int ELEVATOR_SLAVE = 10;
    constexpr int SHOOTER = 11;

    namespace Swerve {
      constexpr int FL_DRIVE = 8;
      constexpr int FL_TURN = 7;

      constexpr int FR_DRIVE = 6;
      constexpr int FR_TURN = 5;

      constexpr int BL_DRIVE = 2;
      constexpr int BL_TURN = 1;

      constexpr int BR_DRIVE = 4;
      constexpr int BR_TURN = 3;
    } // Swerve namespace

    constexpr int AIMER = 12;
    constexpr int INTAKE = 13;
    constexpr int BELT = 14;
  } // CAN namespace

  namespace PWM {}

  namespace AnalogIn {
    constexpr int SWERVE_FL_ENCODER = 3;
    constexpr int SWERVE_FR_ENCODER = 2;
    constexpr int SWERVE_BL_ENCODER = 0;
    constexpr int SWERVE_BR_ENCODER = 1;
  }

} // Constants namespace