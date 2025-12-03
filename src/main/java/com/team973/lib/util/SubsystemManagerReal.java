package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import com.team973.lib.devices.GreyPigeon;
import java.util.concurrent.atomic.AtomicBoolean;

public class SubsystemManagerReal extends SubsystemManager {
  private final GreyPigeon m_pigeon;

  public SubsystemManagerReal(RobotInfo robotInfo, Logger logger) {
    super(robotInfo, logger);

    m_pigeon = new GreyPigeon(logger.subLogger("pigeon"));
  }

  public DriveController initDriveController(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff) {
    return new DriveController(
        logger,
        readyToScore,
        readyToBackOff,
        new SwerveModule(
            0, getRobotInfo().DRIVE_INFO.frontLeftConstants(), logger.subLogger("swerve/mod0")),
        new SwerveModule(
            1, getRobotInfo().DRIVE_INFO.frontRightConstants(), logger.subLogger("swerve/mod1")),
        new SwerveModule(
            2, getRobotInfo().DRIVE_INFO.backLeftConstants(), logger.subLogger("swerve/mod2")),
        new SwerveModule(
            3, getRobotInfo().DRIVE_INFO.backRightConstants(), logger.subLogger("swerve/mod3")),
        m_pigeon);
  }

  public void simulationUpdate() {}
}
