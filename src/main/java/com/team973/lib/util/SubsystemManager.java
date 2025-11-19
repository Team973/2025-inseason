package com.team973.lib.util;

import com.team973.frc2025.Robot;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class SubsystemManager {
  private final RobotInfo m_robotInfo;
  private final Logger m_logger;

  protected SubsystemManager(RobotInfo robotInfo, Logger logger) {
    m_robotInfo = robotInfo;
    m_logger = logger;
  }

  protected RobotInfo getRobotInfo() {
    return m_robotInfo;
  }

  protected Logger getLogger() {
    return m_logger;
  }

  public static SubsystemManager init(RobotInfo robotInfo, Logger logger) {
    if (Robot.isReal()) {
      return new SubsystemManagerReal(robotInfo, logger);
    }

    return new SubsystemManagerSim(robotInfo, logger);
  }

  public abstract DriveController initDriveController(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff);

  public abstract void runSim();
}
