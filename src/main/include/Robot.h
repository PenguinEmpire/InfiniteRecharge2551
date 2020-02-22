/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <math.h>

#include <frc/TimedRobot.h>

#include "AHRS.h"
#include "ctre/Phoenix.h"

#include "frc/Joystick.h"
#include "frc/SPI.h"
#include "frc/Timer.h"

#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h"
#include "frc/trajectory/TrajectoryConfig.h"

#include "PenguinUtil.h"
#include "SwerveDrive.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  frc::Timer m_timer; 
  frc::Joystick m_leftJoystick{0};
  frc::Joystick m_rightJoystick{1};
  frc::Joystick m_gamerJoystick{2};
  
  SwerveDrive m_drivetrain;

  void ProcessJoysticks();
  void Drive();

  WPI_TalonSRX elevator{0};

 private:

  frc::TrajectoryConfig trajectoryConfig{m_drivetrain.K_MAX_VELOCITY, m_drivetrain.K_MAX_ACCELERATION};
  frc::Trajectory exampleTrajectory{std::vector<frc::Trajectory::State>()};
};
