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

  const units::inch_t TRACKWIDTH = units::inch_t(20.75);
  const units::inch_t WHEELBASE = units::inch_t(25.5);

 private:
  AHRS* m_navX = new AHRS(frc::SPI::Port::kMXP);

  const units::inch_t HYPOT_in = units::math::hypot(TRACKWIDTH, WHEELBASE);
  const double HYPOT = hypot(WHEELBASE.to<double>(), TRACKWIDTH.to<double>());

  const units::inch_t CHASSIS_WIDTH = units::inch_t(33.5);
  const units::inch_t CHASSIS_LENGTH = units::inch_t(38.25);

  const units::radian_t FRONT_LEFT_ANGLE_OFFSET  = -units::radian_t(336.0_deg); //   //315.1_deg);
  const units::radian_t FRONT_RIGHT_ANGLE_OFFSET = -units::radian_t(16.8_deg); //   //105.6_deg);
  const units::radian_t BACK_LEFT_ANGLE_OFFSET   = -units::radian_t(271.25_deg - 180_deg); // //90.1_deg);
  const units::radian_t BACK_RIGHT_ANGLE_OFFSET  = -units::radian_t(136.9_deg); //   //63.7_deg);

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
    PenguinConstants::AnalogIn::SWERVE_FL_ENCODER,
    FRONT_LEFT_ANGLE_OFFSET,
    PenguinConstants::CAN::Swerve::FL_DRIVE,
    PenguinConstants::CAN::Swerve::FL_TURN,
    SwerveModuleName("f", "l")
  }; 
  SwerveModule m_frontRightModule{
    FRONT_RIGHT_LOCATION,
    PenguinConstants::AnalogIn::SWERVE_FR_ENCODER,
    FRONT_RIGHT_ANGLE_OFFSET,
    PenguinConstants::CAN::Swerve::FR_DRIVE,
    PenguinConstants::CAN::Swerve::FR_TURN,
    SwerveModuleName("f", "r")
  };
  SwerveModule m_backLeftModule  {
    BACK_LEFT_LOCATION,
    PenguinConstants::AnalogIn::SWERVE_BL_ENCODER,
    BACK_LEFT_ANGLE_OFFSET,
    PenguinConstants::CAN::Swerve::BL_DRIVE,
    PenguinConstants::CAN::Swerve::BL_TURN,
    SwerveModuleName("b", "l")
  };
  SwerveModule m_backRightModule {
    BACK_RIGHT_LOCATION,
    PenguinConstants::AnalogIn::SWERVE_BR_ENCODER,
    BACK_RIGHT_ANGLE_OFFSET,
    PenguinConstants::CAN::Swerve::BR_DRIVE,
    PenguinConstants::CAN::Swerve::BR_TURN,
    SwerveModuleName("b", "r")
  };

  frc::Rotation2d GetAngleAsRot() const;
};