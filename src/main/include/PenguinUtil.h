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
  
  inline constexpr double PI = wpi::math::pi;
  inline constexpr double TWO_PI = 2.0 * wpi::math::pi;
  inline constexpr units::radian_t PI_RAD = units::radian_t(PI);
  inline constexpr units::radian_t TWO_PI_RAD = units::radian_t(TWO_PI);
  inline constexpr frc::Rotation2d ZERO_ROT = frc::Rotation2d();
  inline const frc::Rotation2d PI_ROT = frc::Rotation2d(PI_RAD);
  inline const frc::Rotation2d TWO_PI_ROT = frc::Rotation2d(TWO_PI_RAD);
} // PenguinUtil namespace