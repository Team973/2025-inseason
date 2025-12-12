package com.team973.lib.util;

import com.team973.frc2025.Robot;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.CANdleManger;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.ElevatorIO;
import com.team973.lib.devices.GreyPigeonIO;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class SubsystemManager {
  private final RobotInfo m_robotInfo;
  private final Logger m_logger;

  private final AtomicBoolean m_readyToScore;
  private final AtomicBoolean m_readyToBackOff;

  private final CANdleManger m_candleManager;

  protected SubsystemManager(RobotInfo robotInfo, Logger logger) {
    m_robotInfo = robotInfo;
    m_logger = logger;

    m_readyToScore = new AtomicBoolean(false);
    m_readyToBackOff = new AtomicBoolean(false);

    m_candleManager = new CANdleManger(new Logger("candle manger", 0.2));
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

  public AtomicBoolean getReadyToScore() {
    return m_readyToScore;
  }

  public AtomicBoolean getReadyToBackOff() {
    return m_readyToBackOff;
  }

  public CANdleManger getCANdleManager() {
    return m_candleManager;
  }

  public abstract GreyPigeonIO getPigeon();

  public abstract DriveController getDriveController();

  public abstract ElevatorIO getElevator();

  public void log() {
    m_logger.log("components", new Pose3d[] {getElevator().getPose()});
  }
}
