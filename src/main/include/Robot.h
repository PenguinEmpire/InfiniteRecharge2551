#pragma once

#include <string>
#include <math.h>

#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Limelight.h"

#include "AHRS.h"
#include "ctre/Phoenix.h"

#include "frc/Joystick.h"
#include "frc/SPI.h"
#include "frc2/Timer.h"
#include "frc/Spark.h"

#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h"
#include "frc/trajectory/TrajectoryConfig.h"

#include "LimelightAutonomous.h"
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

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string trajectoryAutoName = "trajectoryAutoTest";
  const std::string limelightAutoName = "limelightAutoTest";
  std::string m_autoSelected;


  Limelight limelight;

  frc2::Timer m_timer;
  units::second_t m_currentTime = units::second_t(0);

  frc::Joystick m_leftJoystick{0};
  frc::Joystick m_rightJoystick{1};
  frc::Joystick m_gamerJoystick{2};
  
  SwerveDrive m_drivetrain;

  void ProcessJoysticks();
  void Drive();

  WPI_TalonSRX elevator{PenguinConstants::CAN::ELEVATOR_MASTER};
  WPI_VictorSPX elevatorHelper{PenguinConstants::CAN::ELEVATOR_SLAVE};

  WPI_TalonSRX shooter{PenguinConstants::CAN::SHOOTER};

  frc::Spark belt{PenguinConstants::PWM::BELT};


  LimelightAutonomous limelightAuto{&limelight};

 private:

  frc::TrajectoryConfig trajectoryConfig{m_drivetrain.K_MAX_VELOCITY, m_drivetrain.K_MAX_ACCELERATION};
  frc::Trajectory exampleTrajectory{std::vector<frc::Trajectory::State>()};
};

