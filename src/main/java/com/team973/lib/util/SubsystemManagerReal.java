package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import java.util.concurrent.atomic.AtomicBoolean;

public class SubsystemManagerReal extends SubsystemManager {
  public SubsystemManagerReal(RobotInfo robotInfo) {
    super(robotInfo);
  }

  public DriveController initDriveController(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff) {
    return new DriveController(
        logger,
        readyToScore,
        readyToBackOff,
        new SwerveModule(
            0, getRobotInfo().DRIVE_INFO.FRONT_LEFT_CONSTANTS, logger.subLogger("swerve/mod0")),
        new SwerveModule(
            1, getRobotInfo().DRIVE_INFO.FRONT_RIGHT_CONSTANTS, logger.subLogger("swerve/mod1")),
        new SwerveModule(
            2, getRobotInfo().DRIVE_INFO.BACK_LEFT_CONSTANTS, logger.subLogger("swerve/mod2")),
        new SwerveModule(
            3, getRobotInfo().DRIVE_INFO.BACK_RIGHT_CONSTANTS, logger.subLogger("swerve/mod3")));
  }
}
