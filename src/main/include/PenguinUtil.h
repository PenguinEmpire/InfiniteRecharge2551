#pragma once

#include <math.h>
#include <wpi/math>

#include <units/units.h>
#include "frc/geometry/Rotation2d.h"
#include "frc/I2C.h"

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

  /** Returns true when @param value is within @param percentTolerance percent of @param reference. */
  inline bool withinPercentTolerance(double value, double reference, double percentTolerance) {
    return approxEqual(value, reference, reference * percentTolerance / 100);
  }
 
  constexpr double PI = wpi::math::pi; // TODO: inline only on functions?
  constexpr double TWO_PI = 2.0 * wpi::math::pi;
  constexpr units::radian_t PI_RAD = units::radian_t(PI);
  constexpr units::radian_t TWO_PI_RAD = units::radian_t(TWO_PI);
  constexpr frc::Rotation2d ZERO_ROT = frc::Rotation2d();
  const frc::Rotation2d PI_ROT = frc::Rotation2d(PI_RAD);
  const frc::Rotation2d TWO_PI_ROT = frc::Rotation2d(TWO_PI_RAD);

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
  constexpr units::second_t DT = 20_ms; // loop period
  constexpr bool DEV_TESTING = true;

  namespace Limelight {
    constexpr double DEFAULT_VALUE = 0.0;
    constexpr units::inch_t HEIGHT = 38.5_in;
  } // Limelight namespace

  namespace CAN {
    namespace Swerve {
      constexpr int FL_DRIVE = 6;
      constexpr int FL_TURN = 5;

      constexpr int FR_DRIVE = 8;
      constexpr int FR_TURN = 7;

      constexpr int BL_DRIVE = 2;
      constexpr int BL_TURN = 1;

      constexpr int BR_DRIVE = 3;
      constexpr int BR_TURN = 4;
    } // Swerve namespace

    constexpr int ELEVATOR_MASTER = 16;
    constexpr int ELEVATOR_SLAVE = 11;
    constexpr int SHOOTER = 20;
    constexpr int AIMER = 15;
    constexpr int INTAKE = 12;
    constexpr int BELT = 9;
    // constexpr int CENTERER = 14;
  } // CAN namespace

  namespace PWM {}

  namespace AnalogIn {
    constexpr int SWERVE_FL_ENCODER = 3;
    constexpr int SWERVE_FR_ENCODER = 2;
    constexpr int SWERVE_BL_ENCODER = 0;
    constexpr int SWERVE_BR_ENCODER = 1;
  }

  namespace DIO {
    constexpr int BELT_ENCODER_A = 1;
    constexpr int BELT_ENCODER_B = 2;

    constexpr int ELEVATOR_ENCODER_A = 6;
    constexpr int ELEVATOR_ENCODER_B = 7;
  } // DIO namespace
  
  namespace I2C {
    constexpr frc::I2C::Port BALL_LIDAR = frc::I2C::Port::kOnboard;
  } // I2C namespace

  namespace ElevatorControl { // all placeholders: TODO
    constexpr units::meters_per_second_t MAX_VEL = 1.75_mps;
    constexpr units::meters_per_second_squared_t MAX_ACCEL = 0.75_mps_sq;

    // Profiled PID constants. Again, real/correct values all TODO
    constexpr double P{1.3};
    constexpr double I{0};
    constexpr double D{0.3};
  } // ElevatorControl namespace

  namespace ShooterSystem {
    namespace ShooterPID { // TODO: not final values
      constexpr double P = 0.000782;
      constexpr double I = 1e-6;
      constexpr double IZone = 1e-5; // maybe?
      constexpr double D = 0;
    } // ShooterPID namespace

    namespace Characterization {
      // Unit definitions copied [from WPILib](https://github.com/wpilibsuite/allwpilib/blob/0ec8ed6c052b18402924daac591ff9e192695825/wpilibc/src/main/native/include/frc/controller/SimpleMotorFeedforward.h#L21):
      using Velocity = units::compound_unit<units::turns, units::inverse<units::seconds>>;
      using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
      using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
      using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;

      constexpr units::volt_t kS = 0.00116 * (1_V);
      constexpr units::unit_t<kv_unit> kV = 0.187 * units::unit_t<kv_unit>(1);
      constexpr units::unit_t<ka_unit> kA = 0.0867 * units::unit_t<ka_unit>(1);
    }

    constexpr double GEAR_RATIO = 30 / 24;
    
    /**  The distance the lidar should (approximately) read if there are balls in front of it.
     * This should be the smallest value it will ever reasonably read with no ball, not the average value.
     * TODO: 30 inches seems about right - needs more testing
     */
    constexpr units::inch_t LIDAR_NORMAL_DISTANCE = 30_in;
  } // ShooterSystem namespace

  namespace Joysticks {
    namespace Gamer {
      namespace Buttons {
        constexpr int A = 1;
        constexpr int B = 2;
        constexpr int Y = 4;
        constexpr int X = 3;
        constexpr int LEFT_STICK = 9;
        constexpr int RIGHT_STICK = 10;
        constexpr int START = 8;
        constexpr int BACK = 7;
        constexpr int LEFT_BUMPER = 5;
        constexpr int RIGHT_BUMPER = 6;
      } // Buttons namespace

      namespace Axes {
        constexpr int LEFT_Y = 1;
        constexpr int LEFT_X = 0;
        constexpr int RIGHT_Y = 5;
        constexpr int RIGHT_X = 4;
        constexpr int LEFT_TRIGGER = 2;
        constexpr int RIGHT_TRIGGER = 3;
      } // Axes namespace
    } // Gamer namespace

    namespace Flight { // TODO
      namespace Buttons {
        constexpr int UPPER_TOP_LEFT = 5;
        constexpr int UPPER_BOTTOM_LEFT = 3;
        constexpr int UPPER_TOP_RIGHT = 6;
        constexpr int UPPER_BOTTOM_RIGHT = 4;
        constexpr int TRIGGER = 1;
        constexpr int SIDE = 2;
        constexpr int LOWER_BOTTOM_LEFT = 11;
        constexpr int LOWER_BOTTOM_RIGHT = 12;
        constexpr int LOWER_MID_LEFT = 9;
        constexpr int LOWER_MID_RIGHT = 10;
        constexpr int LOWER_TOP_LEFT = 7;
        constexpr int LOWER_TOP_RIGHT = 8;
      } // Buttons namespace

      namespace Axes {
        constexpr int Y = 1;
        constexpr int X = 0; // TODO
        constexpr int ROT = 2; // TODO
        constexpr int TRIM = 3; // TODO
      } // Axes namespace
    }
  }
} // Constants namespace