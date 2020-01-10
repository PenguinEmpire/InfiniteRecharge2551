/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

/** 
 * Limelight documentation is available [here](http://docs.limelightvision.io/en/latest/index.html).
 */

class Limelight {
 public:
  Limelight();
  void SetVisionCamMode();
  void SetDriveCamMode();

 private:
  std::shared_ptr<NetworkTable> table;
};