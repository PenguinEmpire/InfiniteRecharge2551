/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <units/units.h>
#include "AHRS.h"

#include "frc/geometry/Translation2d.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include "SwerveModule.h"
#include "SwerveModuleName.h"

class SwerveDrive {
 public:

  SwerveDrive();

  void Drive(double fwd, double str, double rot, bool fieldOriented, frc::Translation2d centerOfRotation);
  void Drive(units::meters_per_second_t fwd, units::meters_per_second_t str, units::radians_per_second_t rot, bool fieldOriented, frc::Translation2d centerOfRotation = frc::Translation2d());
  void Update();
  void UpdateModuleEncoderOFfsetAngles();
  void ResetGyroscope();
  units::degree_t GetAngle() const;
  frc::Pose2d m_location;
  void PutDiagnostics();

  const units::meters_per_second_t K_MAX_VELOCITY = 3.5_mps; // TODO: make accurate. flag: CONTROL_VELOCITY_DIRECTLY
  const units::meters_per_second_squared_t K_MAX_ACCELERATION = units::meters_per_second_squared_t(2); // TODO: make accurate
  const units::radians_per_second_t K_MAX_ANGULAR_VELOCITY = units::radians_per_second_t(2.5);


 private:
  AHRS* m_navX = new AHRS(frc::SPI::Port::kMXP);

 public:
  const units::inch_t TRACKWIDTH = units::inch_t(21.25);
  const units::inch_t WHEELBASE = units::inch_t(24);
 private:
  const units::inch_t HYPOT_in = units::math::hypot(TRACKWIDTH, WHEELBASE);
  const double HYPOT = hypot(WHEELBASE.to<double>(), TRACKWIDTH.to<double>());

  const units::inch_t CHASSIS_WIDTH = units::inch_t(27.625);
  const units::inch_t CHASSIS_LENGTH = units::inch_t(32.25);

  const units::radian_t FRONT_LEFT_ANGLE_OFFSET  = -units::radian_t(315.1_deg);
  const units::radian_t FRONT_RIGHT_ANGLE_OFFSET = -units::radian_t(105.6_deg);
  const units::radian_t BACK_LEFT_ANGLE_OFFSET   = -units::radian_t(90.1_deg);
  const units::radian_t BACK_RIGHT_ANGLE_OFFSET  = -units::radian_t(63.7_deg);

  const frc::Translation2d FRONT_LEFT_LOCATION {+TRACKWIDTH / 2.0, +WHEELBASE / 2.0};
  const frc::Translation2d FRONT_RIGHT_LOCATION{+TRACKWIDTH / 2.0, -WHEELBASE / 2.0}; 
  const frc::Translation2d BACK_LEFT_LOCATION  {-TRACKWIDTH / 2.0, +WHEELBASE / 2.0}; 
  const frc::Translation2d BACK_RIGHT_LOCATION {-TRACKWIDTH / 2.0, -WHEELBASE / 2.0};

 public:
  const frc::Translation2d FRONT_LEFT_CORNER_LOCATION {+CHASSIS_WIDTH / 2.0, +CHASSIS_LENGTH / 2.0};
  const frc::Translation2d FRONT_RIGHT_CORNER_LOCATION{+CHASSIS_WIDTH / 2.0, -CHASSIS_LENGTH / 2.0};
  const frc::Translation2d BACK_LEFT_CORNER_LOCATION  {-CHASSIS_WIDTH / 2.0, +CHASSIS_LENGTH / 2.0};
  const frc::Translation2d BACK_RIGHT_CORNER_LOCATION {-CHASSIS_WIDTH / 2.0, -CHASSIS_LENGTH / 2.0};

  frc::SwerveDriveKinematics<4> m_kinematics{
    FRONT_LEFT_LOCATION,
    FRONT_RIGHT_LOCATION,
    BACK_LEFT_LOCATION,
    BACK_RIGHT_LOCATION
  };

 private:
  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, PenguinUtil::ZERO_ROT}; // WPILib has `GetAngleAsRot()` for the starting angle, but I don't feel like defining it in the header file and we're resetting anyway.

  SwerveModule m_frontLeftModule {
    FRONT_LEFT_LOCATION,
    PenguinConstants::Swerve::FL::ANALOG_ENCODER_PORT,
    FRONT_LEFT_ANGLE_OFFSET,
    PenguinConstants::Swerve::FL::DRIVE_MOTOR_CAN_ID,
    PenguinConstants::Swerve::FL::TURN_MOTOR_CAN_ID,
    SwerveModuleName("f", "l")
  }; 
  SwerveModule m_frontRightModule{
    FRONT_RIGHT_LOCATION,
    PenguinConstants::Swerve::FR::ANALOG_ENCODER_PORT,
    FRONT_RIGHT_ANGLE_OFFSET,
    PenguinConstants::Swerve::FR::DRIVE_MOTOR_CAN_ID,
    PenguinConstants::Swerve::FR::TURN_MOTOR_CAN_ID,
    SwerveModuleName("f", "r")
  };
  SwerveModule m_backLeftModule  {
    BACK_LEFT_LOCATION,
    PenguinConstants::Swerve::BL::ANALOG_ENCODER_PORT,
    BACK_LEFT_ANGLE_OFFSET,
    PenguinConstants::Swerve::BL::DRIVE_MOTOR_CAN_ID,
    PenguinConstants::Swerve::BL::TURN_MOTOR_CAN_ID,
    SwerveModuleName("b", "l")
  };
  SwerveModule m_backRightModule {
    BACK_RIGHT_LOCATION,
    PenguinConstants::Swerve::BR::ANALOG_ENCODER_PORT,
    BACK_RIGHT_ANGLE_OFFSET,
    PenguinConstants::Swerve::BR::DRIVE_MOTOR_CAN_ID,
    PenguinConstants::Swerve::BR::TURN_MOTOR_CAN_ID,
    SwerveModuleName("b", "r")
  };

  frc::Rotation2d GetAngleAsRot() const;
};