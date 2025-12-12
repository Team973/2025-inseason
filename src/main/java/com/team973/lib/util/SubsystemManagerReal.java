package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Elevator;
import com.team973.frc2025.subsystems.ElevatorIO;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.devices.GreyPigeonIO;

public class SubsystemManagerReal extends SubsystemManager {
  private final GreyPigeonIO m_pigeon;
  private final DriveController m_driveController;
  private final ElevatorIO m_elevator;

  public SubsystemManagerReal(RobotInfo robotInfo, Logger logger) {
    super(robotInfo, logger);

    m_pigeon = new GreyPigeon(logger.subLogger("pigeon"));

    Logger driveLogger = logger.subLogger("drive", 0.05);

    m_driveController =
        new DriveController(
            logger,
            getReadyToScore(),
            getReadyToBackOff(),
            new SwerveModule(
                0,
                getRobotInfo().DRIVE_INFO.FRONT_LEFT_CONSTANTS,
                driveLogger.subLogger("swerve/mod0")),
            new SwerveModule(
                1,
                getRobotInfo().DRIVE_INFO.FRONT_RIGHT_CONSTANTS,
                driveLogger.subLogger("swerve/mod1")),
            new SwerveModule(
                2,
                getRobotInfo().DRIVE_INFO.BACK_LEFT_CONSTANTS,
                driveLogger.subLogger("swerve/mod2")),
            new SwerveModule(
                3,
                getRobotInfo().DRIVE_INFO.BACK_RIGHT_CONSTANTS,
                driveLogger.subLogger("swerve/mod3")),
            m_pigeon);

    m_elevator = new Elevator(logger.subLogger("elevator", 0.2), getCANdleManager(), robotInfo);
  }

  public GreyPigeonIO getPigeon() {
    return m_pigeon;
  }

  public DriveController getDriveController() {
    return m_driveController;
  }

  public ElevatorIO getElevator() {
    return m_elevator;
  }
}
