/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <wpi/math>
#include "frc/smartdashboard/SmartDashboard.h"

#include <units/units.h>

#include "SwerveModule.h"

using SD = frc::SmartDashboard;

SwerveModule::SwerveModule(frc::Translation2d pos, int analogEncoderPort, units::radian_t analogEncoderOffset, int driveMotorCANID, int turnMotorCANID, SwerveModuleName moduleName)
                          : m_driveMotor(driveMotorCANID, rev::CANSparkMax::MotorType::kBrushless), 
                            m_turnMotor(turnMotorCANID, rev::CANSparkMax::MotorType::kBrushless),
                            m_moduleName{moduleName},
                            m_turnEncoder{analogEncoderPort, analogEncoderOffset},
                            m_modulePosition{pos} {

  m_driveEncoder.SetPositionConversionFactor(WHEEL_DIAMETER * wpi::math::pi / DRIVE_REDUCTION);
  m_driveEncoder.SetVelocityConversionFactor(WHEEL_DIAMETER * wpi::math::pi / DRIVE_REDUCTION * (1.0 / 60.0)); // SDS: "RPM to units per sec"
                            }

void SwerveModule::PutDiagnostics() {
  SD::PutNumber(m_moduleName.GetFullTitle() + " Angle", GetCurrentAngle().to<double>());
}

void SwerveModule::UpdateState() {
  // TODO
}

units::radian_t SwerveModule::GetCurrentAngle() {
  return units::radian_t(wpi::math::pi); // TODO: actually return current angle
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
  units::meters_per_second_t speed = state.speed;
  frc::Rotation2d angle = state.angle;

  const frc::Rotation2d rot_pi = frc::Rotation2d(units::radian_t(wpi::math::pi));

  if (speed < 0_mps) {
    speed *= -1;
    angle.RotateBy(rot_pi);
  }
  
  /** SDS has:
   *   angle %= 2.0 * Math.PI;
       if (angle < 0.0) {
           angle += 2.0 * Math.PI;
       }
   *
   * Assuming/hoping that frc::Rotation2d::Degrees() automatically returns the terminal (?) angle (within `[0, 2pi)`).
   * Update: pretty sure it doesn't.
   */

  double temp_angle = angle.Degrees().to<double>();
  temp_angle = fmod(temp_angle, 360);
  if (temp_angle < 0) {
    angle.RotateBy(rot_pi);
  }

  SDS_targetSpeed = speed.to<double>();
  SDS_targetAngle = angle.Radians().to<double>();
}