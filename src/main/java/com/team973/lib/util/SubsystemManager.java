package com.team973.lib.util;

import com.team973.frc2025.Robot;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class SubsystemManager {
  private final RobotInfo m_robotInfo;

  protected SubsystemManager(RobotInfo robotInfo) {
    m_robotInfo = robotInfo;
  }

  protected RobotInfo getRobotInfo() {
    return m_robotInfo;
  }

  public static SubsystemManager init(RobotInfo robotInfo) {
    if (Robot.isReal()) {
      return new SubsystemManagerReal(robotInfo);
    }

    return new SubsystemManagerSim(robotInfo);
  }

  public abstract DriveController initDriveController(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff);
}
