package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModuleSim;
import java.util.concurrent.atomic.AtomicBoolean;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SubsystemManagerSim extends SubsystemManager {
  private final SwerveDriveSimulation m_swerveDriveSimulation;

  public SubsystemManagerSim(RobotInfo robotInfo) {
    super(robotInfo);

    m_swerveDriveSimulation =
        new SwerveDriveSimulation(
            robotInfo.DRIVE_INFO.getDriveTrainSimulationConfig(),
            robotInfo.DRIVE_INFO.getSimStartingPose());
  }

  public DriveController initDriveController(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff) {
    return new DriveController(
        logger,
        readyToScore,
        readyToBackOff,
        new SwerveModuleSim(
            0,
            m_swerveDriveSimulation.getModules()[0],
            getRobotInfo().DRIVE_INFO.FRONT_LEFT_CONSTANTS,
            logger.subLogger("swerve/mod0")),
        new SwerveModuleSim(
            1,
            m_swerveDriveSimulation.getModules()[1],
            getRobotInfo().DRIVE_INFO.FRONT_RIGHT_CONSTANTS,
            logger.subLogger("swerve/mod1")),
        new SwerveModuleSim(
            2,
            m_swerveDriveSimulation.getModules()[2],
            getRobotInfo().DRIVE_INFO.BACK_LEFT_CONSTANTS,
            logger.subLogger("swerve/mod2")),
        new SwerveModuleSim(
            3,
            m_swerveDriveSimulation.getModules()[3],
            getRobotInfo().DRIVE_INFO.BACK_RIGHT_CONSTANTS,
            logger.subLogger("swerve/mod3")));
  }
}
