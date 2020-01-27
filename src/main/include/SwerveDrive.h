/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>
#include "AHRS.h"

#include "frc/geometry/Translation2d.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"

class SwerveDrive {
 public:
  SwerveDrive();

  void Drive(double fwd, double str, double rot, bool fieldOriented);


  AHRS* m_navX = new AHRS(frc::SPI::Port::kMXP);
  void ResetGyroscope();



 private:
  const units::inch_t TRACKWIDTH = 21.25_in;
  const units::inch_t WHEELBASE = 24_in;

  const units::radian_t FRONT_LEFT_ANGLE_OFFSET  = -units::radian_t(134.5_deg);
  const units::radian_t FRONT_RIGHT_ANGLE_OFFSET = -units::radian_t(122.4_deg);
  const units::radian_t BACK_LEFT_ANGLE_OFFSET   = -units::radian_t(268.7_deg);
  const units::radian_t BACK_RIGHT_ANGLE_OFFSET  = -units::radian_t(243.6_deg);

  const frc::Translation2d FRONT_LEFT_LOCATION {+TRACKWIDTH / 2.0, +WHEELBASE / 2.0};
  const frc::Translation2d FRONT_RIGHT_LOCATION{+TRACKWIDTH / 2.0, -WHEELBASE / 2.0}; 
  const frc::Translation2d BACK_LEFT_LOCATION  {-TRACKWIDTH / 2.0, +WHEELBASE / 2.0}; 
  const frc::Translation2d BACK_RIGHT_LOCATION {-TRACKWIDTH / 2.0, -WHEELBASE / 2.0};

  frc::SwerveDriveKinematics<4> m_kinematics{
    FRONT_LEFT_LOCATION,
    FRONT_RIGHT_LOCATION,
    BACK_LEFT_LOCATION,
    BACK_RIGHT_LOCATION
  };

  SwerveModule m_frontLeftModule {
    FRONT_LEFT_LOCATION,
    3, // analog input port. TODO: factor out into constants file/namespace/class.
    FRONT_LEFT_ANGLE_OFFSET};
  SwerveModule m_frontRightModule{
    FRONT_RIGHT_LOCATION,
    2,
    FRONT_RIGHT_ANGLE_OFFSET};
  SwerveModule m_backLeftModule  {
    BACK_LEFT_LOCATION,
    0,
    BACK_LEFT_ANGLE_OFFSET};
  SwerveModule m_backRightModule {
    BACK_RIGHT_LOCATION,
    1,
    BACK_RIGHT_ANGLE_OFFSET};

};