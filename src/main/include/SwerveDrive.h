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

  enum ModuleLocation {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
    NONE // TODO: this seems bad and unclear
  };


  SwerveDrive();

  void Drive(double fwd, double str, double rot, bool fieldOriented, SwerveDrive::ModuleLocation centerOfRotation);
  void Drive(units::meters_per_second_t fwd, units::meters_per_second_t str, units::radians_per_second_t rot, bool fieldOriented, frc::Translation2d centerOfRotation);
  void Update();

  AHRS* m_navX = new AHRS(frc::SPI::Port::kMXP);
  void ResetGyroscope();
  void PutDiagnostics();


 private:
  const units::inch_t TRACKWIDTH = units::inch_t(21.25);
  const units::inch_t WHEELBASE = units::inch_t(24);
  const double HYPOT = hypot(WHEELBASE.to<double>(), TRACKWIDTH.to<double>());

  const units::inch_t CHASSIS_WIDTH = units::inch_t(27.625);
  const units::inch_t CHASSIS_LENGTH = units::inch_t(32.25);

  const units::meters_per_second_t K_MAX_VELOCITY = 2.5_mps;
  const units::radians_per_second_t K_MAX_ANGULAR_VELOCITY = units::radians_per_second_t(2.5);

  const units::radian_t FRONT_LEFT_ANGLE_OFFSET  = -units::radian_t(134.9_deg); // 134.5_deg);
  const units::radian_t FRONT_RIGHT_ANGLE_OFFSET = -units::radian_t(148.3_deg); // 150.0_deg);
  const units::radian_t BACK_LEFT_ANGLE_OFFSET   = -units::radian_t(270.05_deg); // 268.7_deg);
  const units::radian_t BACK_RIGHT_ANGLE_OFFSET  = -units::radian_t(243.63_deg); // 243.6_deg);

  const frc::Translation2d FRONT_LEFT_LOCATION {+TRACKWIDTH / 2.0, +WHEELBASE / 2.0};
  const frc::Translation2d FRONT_RIGHT_LOCATION{+TRACKWIDTH / 2.0, -WHEELBASE / 2.0}; 
  const frc::Translation2d BACK_LEFT_LOCATION  {-TRACKWIDTH / 2.0, +WHEELBASE / 2.0}; 
  const frc::Translation2d BACK_RIGHT_LOCATION {-TRACKWIDTH / 2.0, -WHEELBASE / 2.0};

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

  SwerveModule m_frontLeftModule {
    FRONT_LEFT_LOCATION,
    3, // analog input port. TODO: factor out into constants file/namespace/class.
    FRONT_LEFT_ANGLE_OFFSET,
    8, 7, // CAN ID of drive and angle motors. TODO: factor out into constants file/etc
    SwerveModuleName("f", "l")}; 
  SwerveModule m_frontRightModule{
    FRONT_RIGHT_LOCATION,
    2,
    FRONT_RIGHT_ANGLE_OFFSET,
    6, 5,
    SwerveModuleName("f", "r")};
  SwerveModule m_backLeftModule  {
    BACK_LEFT_LOCATION,
    0,
    BACK_LEFT_ANGLE_OFFSET,
    2, 1,
    SwerveModuleName("b", "l")};
  SwerveModule m_backRightModule {
    BACK_RIGHT_LOCATION,
    1,
    BACK_RIGHT_ANGLE_OFFSET,
    4, 3,
    SwerveModuleName("b", "r")};

};