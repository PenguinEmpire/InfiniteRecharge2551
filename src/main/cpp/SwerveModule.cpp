/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <wpi/math>

#include "SwerveModule.h"

SwerveModule::SwerveModule(frc::Translation2d pos, int analogEncoderPort, units::radian_t analogEncoderOffset, int driveMotorCANID, int turnMotorCANID)
                          : m_modulePosition{pos},
                            m_turnEncoder{analogEncoderPort, analogEncoderOffset},
                            m_driveMotor(driveMotorCANID, rev::CANSparkMax::MotorType::kBrushless), 
                            m_turnMotor(turnMotorCANID, rev::CANSparkMax::MotorType::kBrushless) {

  m_driveEncoder.SetPositionConversionFactor(WHEEL_DIAMETER * wpi::math::pi / DRIVE_REDUCTION);
  m_driveEncoder.SetVelocityConversionFactor(WHEEL_DIAMETER * wpi::math::pi / DRIVE_REDUCTION * (1.0 / 60.0)); // SDS: "RPM to units per sec"
                            }