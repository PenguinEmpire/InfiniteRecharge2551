/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveModule.h"

SwerveModule::SwerveModule(frc::Translation2d pos, int analogEncoderPort, units::radian_t analogEncoderOffset)
                          : m_modulePosition{pos},
                            m_turnEncoder{analogEncoderPort, analogEncoderOffset} {}