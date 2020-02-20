/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <units/units.h>

#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/drive/Vector2d.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>

#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "rev/CANPIDController.h"

#include "PenguinUtil.h"
#include "TurnEncoder.h"
#include "SwerveModuleName.h"

// using inches_per_sec = units::compound_unit<units::inch_t, units::inverse<units::second_t>>;      

class SwerveModule {
 public:
  SwerveModule(frc::Translation2d pos,
               int analogEncoderPort,
               units::radian_t analogEncoderOffset,
               int driveMotorCANID,
               int turnMotorCANID,
               SwerveModuleName moduleName);

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turnMotor;

  /** @param state: the desired frc::SwerveModuleState, from the kinematics */
  void SetDesiredState(frc::SwerveModuleState& state);
  void SetDirectly(double angle, double speed);

  frc::SwerveModuleState GetState() const;

  void PutSwerveModuleState(std::string, frc::SwerveModuleState&) const;
  void PutSwerveModuleState(std::string, units::degree_t, units::meters_per_second_t) const;
  void PutSwerveModuleState(std::string, double, double) const;

  void PutDiagnostics() const;
  void ReadSensors();

  void UpdateAnalogOffset();

  SwerveModuleName m_moduleName;

  TurnEncoder m_turnEncoder;
  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

  frc::Translation2d m_modulePosition;

  units::radian_t m_currentAngle = 0_rad;
  units::meters_per_second_t m_currentVelocity = 0_mps;

  rev::CANPIDController m_onboardTurnMotorPIDController = m_turnMotor.GetPIDController();

  // copied from SDS
  static constexpr double SDS_DRIVE_REDUCTION = 8.31 / 1.0; // (gear ratio)
  static constexpr units::meter_t SDS_WHEEL_DIAMETER = 4.0_in; // (in)
  static constexpr double SDS_DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * wpi::math::pi)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);
};